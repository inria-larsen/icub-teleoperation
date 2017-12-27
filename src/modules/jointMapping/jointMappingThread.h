/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef JOINT_MAPPING_THREAD
#define JOINT_MAPPING_THREAD

// System includes
#include <cstring>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <map>
#include <set>
#include <Eigen/Dense>
#include <Eigen/Core>


// Yarp includes
#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/PortInfo.h>
#include <yarp/os/Time.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Mutex.h>

// Local includes

using namespace yarp::os;
using namespace std;
using namespace Eigen;

const double PI = 3.141592653589793;


//===============================================
//        JointMapping Main THREAD (runs every 10ms)
//===============================================

class jointMappingThread: public yarp::os::RateThread
{
	/** prefix for all the ports opened by this module */
	std::string moduleName;
	/** prefix for all the ports of the robot at which we are connecting */
	std::string robotName;
	double offset;
	/** helper variable for printing every printPeriod milliseconds */
	int                 printCountdown;
	/** period after which some diagnostic messages are print */
	double              printPeriod;
	yarp::os::Stamp timestamp;
	wbi::wholeBodyInterface& m_robot;
        int actuatedDOFs;
        //Limits
	bool m_checkJointLimits;
        Eigen::VectorXd m_minJointLimits; /* actuatedDOFs */
        Eigen::VectorXd m_maxJointLimits; /* actuatedDOFs */


	void mapping_run()
	{
	    //Compute joints
	    publishJoints();

	    //if normal mode, publish the
	    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

	    if( printCountdown == 0 ) {
	       
		double avgTime, stdDev, avgTimeUsed, stdDevUsed;

		getEstPeriod(avgTime, stdDev);
		getEstUsed(avgTimeUsed, stdDevUsed);

	    }

	}

	yarp::os::Mutex run_mutex;
	bool run_mutex_acquired;
	Eigen::VectorXd jointPos;

	yarp::os::Property yarp_options;
        BufferedPort<Bottle> port; /**<Port for reading and writing the joint values*/

	void getRobotJoints()
	{
	    //read joint angles value from xsens port
            Bottle *input = port.read();  

            double neck_pitch = (input->get(17).asDouble())*-1;
	    double neck_roll = input->get(15).asDouble();
	    double neck_yaw = input->get(16).asDouble();
	    double torso_yaw = (input->get(4).asDouble()+input->get(7).asDouble()+input->get(10).asDouble())*-1;
	    double torso_roll = (input->get(6).asDouble()+input->get(9).asDouble())*-1;
	    double torso_pitch = (input->get(8).asDouble()+input->get(5).asDouble()+input->get(2).asDouble());
	    double l_shoulder_pitch = (input->get(35).asDouble())*-1;
	    double l_shoulder_roll = input->get(33).asDouble();
	    double l_shoulder_yaw = input->get(34).asDouble();
	    double l_elbow = input->get(38).asDouble();
	    double l_wrist_prosup = (input->get(40).asDouble())*-1;
	    double l_wrist_pitch = (input->get(41).asDouble())*-1;
	    double l_wrist_yaw = (input->get(39).asDouble())*-1;
	    double r_shoulder_pitch = (input->get(23).asDouble())*-1;
	    double r_shoulder_roll = input->get(21).asDouble();
	    double r_shoulder_yaw = input->get(22).asDouble();
	    double r_elbow = input->get(26).asDouble();
	    double r_wrist_prosup = (input->get(28).asDouble())*-1;
	    double r_wrist_pitch = (input->get(29).asDouble())*-1;
	    double r_wrist_yaw = (input->get(27).asDouble())*-1;
	    double l_hip_pitch = input->get(56).asDouble();
	    double l_hip_roll = input->get(54).asDouble();
	    double l_hip_yaw = (input->get(55).asDouble())*-1;
	    double l_knee = (input->get(59).asDouble())*-1;
	    double l_ankle_pitch = (input->get(62).asDouble())*-1;
	    double l_ankle_roll = input->get(60).asDouble();
	    double r_hip_pitch = input->get(44).asDouble();
	    double r_hip_roll = input->get(42).asDouble();
	    double r_hip_yaw = (input->get(43).asDouble())*-1;
	    double r_knee = (input->get(47).asDouble())*-1;
	    double r_ankle_pitch = (input->get(50).asDouble())*-1;
	    double r_ankle_roll = input->get(48).asDouble();
		
	    jointPos.resize(32);
	    jointPos << torso_pitch*PI/180, torso_roll*PI/180, torso_yaw*PI/180, neck_pitch*PI/180, neck_roll*PI/180, neck_yaw*PI/180, l_shoulder_pitch*PI/180, l_shoulder_roll*PI/180, l_shoulder_yaw*PI/180, l_elbow*PI/180, l_wrist_prosup*PI/180, l_wrist_pitch*PI/180, l_wrist_yaw*PI/180, r_shoulder_pitch*PI/180, r_shoulder_roll*PI/180, r_shoulder_yaw*PI/180, r_elbow*PI/180, r_wrist_prosup*PI/180, r_wrist_pitch*PI/180, r_wrist_yaw*PI/180, l_hip_pitch*PI/180, l_hip_roll*PI/180, l_hip_yaw*PI/180, l_knee*PI/180, l_ankle_pitch*PI/180, l_ankle_roll*PI/180, r_hip_pitch*PI/180, r_hip_roll*PI/180, r_hip_yaw*PI/180, r_knee*PI/180, r_ankle_pitch*PI/180, r_ankle_roll*PI/180;
	}


/*	void respectJointLimits() 
	{
	    MatrixXd jointLim(2,32);
	    jointLim << -0.51679199, -0.349066, 0.785398, -0.349066, -0.523599, -0.872665, -1.65806, 0, -0.645772, 0.261799, -1.0472, -1.392773, -0.3455752, -1.65806, 0, -0.645772, 0.261799, -1.0472, -1.392773, -0.3455752, -0.610865, -0.261799, -1.22173, -1.74533, -0.523599, -0.349066, -0.610865, -0.261799, -1.22173, -1.74533, -0.523599, -0.349066,
			0.29530971, 0.349066, 0.785398, 1.22173, 0.523599, 0.872665, 0.174533, 2.79253, 1.39626, 1.85005, 1.0472, 0.42149701, 0.43196899, 0.174533, 2.79253, 1.39626, 1.85005, 1.0472, 0.42149701, 0.43196899, 1.48353, 1.5708, 1.22173, 0, 0.523599, 0.349066, 1.48353, 1.5708, 1.22173, 0, 0.523599, 0.349066;
*/

	    /* Edit the JointAngles according to the Joitn limits of the iCub */ 
/*	    for (int i = 0; i < jointPos.size(); i++){ 
	    	if (jointPos(i) <= jointLim(0,i))		//check min
			jointPos(i) = jointLim(0,i) + offset;
		if (jointPos(i) >= jointLim(1,i))		//check max
			jointPos(i) = jointLim(1,i) - offset;
	    }

 	    neck_pitch = jointPos(0);
            neck_roll = jointPos(1); 
	    neck_yaw = jointPos(2);
	    torso_yaw = jointPos(3);
	    torso_roll = jointPos(4);
	    torso_pitch = jointPos(5);
	    l_shoulder_pitch = jointPos(6);
	    l_shoulder_roll = jointPos(7);
     	    l_shoulder_yaw = jointPos(8);
	    l_elbow = jointPos(9);
	    l_wrist_prosup = jointPos(10);
	    l_wrist_pitch = jointPos(11);
	    l_wrist_yaw = jointPos(12);
	    r_shoulder_pitch = jointPos(13);
	    r_shoulder_roll = jointPos(14);
	    r_shoulder_yaw = jointPos(15);
	    r_elbow = jointPos(16);
	    r_wrist_prosup = jointPos(17);
	    r_wrist_pitch = jointPos(18);
	    r_wrist_yaw = jointPos(19);
	    l_hip_pitch = jointPos(20);
	    l_hip_roll = jointPos(21);
	    l_hip_yaw = jointPos(22);
	    l_knee = jointPos(23);
	    l_ankle_pitch = jointPos(24);
	    l_ankle_roll = jointPos(25);
	    r_hip_pitch = jointPos(26);
	    r_hip_roll = jointPos(27);
	    r_hip_yaw = jointPos(28);
	    r_knee = jointPos(29);
	    r_ankle_pitch = jointPos(30);
	    r_ankle_roll = jointPos(31);
	}
*/
	void publishJoints()
	{
	    Bottle& output = port.prepare();
            output.clear();

	    for (int i=0; i < jointPos.size(); i++){
                output.addDouble(jointPos(i));
	    }
	    /* JointAngles coordinates to send to the iCub */
/*	    //output.addDouble(neck_pitch);
	    //output.addDouble(neck_roll);
	    //output.addDouble(neck_yaw);   
	    output.addDouble(torso_yaw);
	    output.addDouble(torso_roll);
	    output.addDouble(torso_pitch);
	    output.addDouble(l_shoulder_pitch);
	    output.addDouble(l_shoulder_roll);
	    output.addDouble(l_shoulder_yaw);
	    output.addDouble(l_elbow);
 	    //output.addDouble(l_wrist_prosup);
	    //output.addDouble(l_wrist_pitch);
	    //output.addDouble(l_wrist_yaw);
	    output.addDouble(r_shoulder_pitch);
	    output.addDouble(r_shoulder_roll);
	    output.addDouble(r_shoulder_yaw);
	    output.addDouble(r_elbow);
 	    //output.addDouble(r_wrist_prosup);
	    //output.addDouble(r_wrist_pitch);
	    //output.addDouble(r_wrist_yaw);
	    output.addDouble(l_hip_pitch);
	    output.addDouble(l_hip_roll);
	    output.addDouble(l_hip_yaw);
	    output.addDouble(l_knee);
	    output.addDouble(l_ankle_pitch);
	    output.addDouble(l_ankle_roll);
	    output.addDouble(r_hip_pitch);
	    output.addDouble(r_hip_roll);
	    output.addDouble(r_hip_yaw);
	    output.addDouble(r_knee);
	    output.addDouble(r_ankle_pitch);
	    output.addDouble(r_ankle_roll);
*/	    port.write();
	}
	    

	public:

	    jointMappingThread(string _name,
				       string _robotName,
				       int _actuatedDOFs,
				       wbi::wholeBodyInterface& robot,
				       yarp::os::Property & _yarp_options,
				       int _period,
				       double _offset)
	    :  RateThread(_period),
	       moduleName(_name),
	       robotName(_robotName),
               actuatedDOFs(_actuatedDOFs),
	       m_robot(robot),
	       yarp_options(_yarp_options),
	       m_checkJointLimits(true),	
	       offset(_offset),
	       printCountdown(0),
	       printPeriod(2000),
	       run_mutex_acquired(false),
	       m_minJointLimits(_actuatedDOFs),
               m_maxJointLimits(_actuatedDOFs)
	{

	    yInfo() << "Launching jointMappingThread with name : " << _name << " and robotName " << _robotName << " and period " << _period;
	   
	}


	bool threadInit()
	{  

 	     //limits
            bool result = false;
            result = m_robot.getJointLimits(m_minJointLimits.data(), m_maxJointLimits.data());
            if (!result) {
                yError("Failed to compute joint limits.");
                return false;
            }

            if (this->m_checkJointLimits) {
                std::cout << "[INFO]Joint limits are:\nmin" <<m_minJointLimits.transpose() << "\nmax " << m_maxJointLimits.transpose() << "\n";
            } else {
                yInfo("Joint limits disabled");
            }


	    //opening port
	    port.open(string("/"+moduleName+"/joint").c_str());

	    yarp::os::Bottle & joint_group = yarp_options.findGroup("JOINT_LIMITS");
	  
	    yInfo() << "jointMappingThread::threadInit finished successfully.";

	    return true;
	}
	   

        //------ RUN ------
	void run()
	{
	    if( this->run_mutex_acquired )
	    {
		yError() << "jointMapping: run_mutex already acquired at the beginning of run method.";
		yError() << "    this could cause some problems, please report an issue at https://github.com/robotology/codyco-modules/issues/new";
	    }

	    run_mutex.lock();
	    this->run_mutex_acquired = true;
	    getRobotJoints();
	    
	    //respectJointLimits();

	    mapping_run();
	   
	    this->run_mutex_acquired = false;
	    run_mutex.unlock();
	}
	   

	void threadRelease()
	{
	    run_mutex.lock();

	    yInfo() << "Closing joint port";
	    port.close();

	    run_mutex.unlock();
	}

};


#endif
