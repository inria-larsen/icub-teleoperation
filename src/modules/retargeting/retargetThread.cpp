/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

// Local includes
#include "retargetThread.h"
#include <math.h>     
// iDynTree includes
#include "iCub/iDynTree/yarp_kdl.h"  

namespace wbi {
    class ID;
    class IDList;
}

using namespace yarp::sig;
using namespace yarp::os;
using namespace wbi;
using namespace yarpWbi;
using namespace std;
using namespace Eigen;

const double PI = 3.141592653589793;


//===============================================
//        Retarget Main THREAD
//===============================================

retargetThread::retargetThread(string _name,
			       string _robotName,
                               int _actuatedDOFs,
			       wbi::wholeBodyInterface& robot,
			       bool checkJointLimits,
			       yarp::os::Property & _yarp_options,
			       int _period,
			       double _offset)
    :  RateThread(_period),
       moduleName(_name),
       period(_period),
       robotName(_robotName),
       actuatedDOFs(_actuatedDOFs),
       m_robot(robot),
       yarp_options(_yarp_options),
       m_checkJointLimits(checkJointLimits),	
       offset(_offset),
       printCountdown(0),
       printPeriod(2000),
       run_mutex_acquired(false),
       m_minJointLimits(_actuatedDOFs),
       m_maxJointLimits(_actuatedDOFs)
{

    yInfo() << "Launching retargetThread with name : " << _name << " and robotName " << _robotName << " and period " << _period;
      if( !_yarp_options.check("urdf") )
       {
	    yError() << "retarget error: urdf not found in configuration files";
	    return;
       }

    std::string urdf_file = _yarp_options.find("urdf").asString().c_str();

    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());

    yarp::os::ResourceFinder rf;
    if( _yarp_options.check("verbose") )
    {
	rf.setVerbose();
    }

    //std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());
    
    yInfo() << "Tryng to open " << urdf_file << " as robot model";
    
    bool ok = icub_model.loadURDFModel(urdf_file_path);

    if( !ok )
    {
        std::cerr << "Loading urdf file " << urdf_file << " failed, exiting" << std::endl;
        return;
    }  
   
}


bool retargetThread::threadInit()
{  
    acc_counter = 0;

    initJoint();
    
    initCoM();
    
    //opening reading ports
    jointRead.open(string("/"+moduleName+"/q:i").c_str());
    posRead.open(string("/"+moduleName+"/pose:i").c_str()); 
  
    yInfo() << "retargetThread::threadInit finished successfully.";

    return true;
}


bool retargetThread::initJoint()
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

    //deg or radiants
    jointFormat = yarp_options.find("joint_output").asString();
    //controller joint list
    joint_list = yarp_options.find("wbi_joint_list").asString();
    //controller
    controller =yarp_options.find("controller").asString();
 
    // Open writing port for joints
    port_joint.open(string("/"+moduleName+"/q:o").c_str()); 
    
    return true;  
}


bool retargetThread::initCoM()
{
    q.resize(icub_model.getNrOfDOFs());
    for (int i=0; i<q.size(); i++){
        q(i)=0;
    }
    icub_model.setAng(q);
    
    link_name = yarp_options.find("reference_frame").asString();
	
    link_index = icub_model.getLinkIndex(link_name);

    // Open writing port for com
    port_com = new BufferedPort<Vector>;
    port_com->open(string("/"+moduleName+"/com:o").c_str());

    return true;
}

void retargetThread::publishJoints()
{
    Bottle& output = port_joint.prepare();
    output.clear();
    output.addDouble(acc_counter);
    for (int i=0; i < jointPos.size(); i++){
        output.addDouble(jointPos(i));
    }
    
    //if (acc_counter>=2){
    port_joint.write();
    //}
}

void retargetThread::publishCom()
{
	// Stream com in index frame
	//KDL::Vector com = icub_model.getCOMKDL();
        yarp::sig::Vector com = icub_model.getCOM(link_index);
       /* 
	dcom.resize(3);
        if (acc_counter>=1) {
		for (int i=0; i<3; i++){
		    dcom(i) = (com(i) - old_com(i))/(period*0.001);
		}
        }
	else {
                for (int i=0; i<3; i++){
		    dcom(i) = 0;
		}
        }

        ddcom.resize(3);
        if (acc_counter>=2) {
		for (int i=0; i<3; i++){
		    ddcom(i) = (dcom(i) - old_dcom(i))/(period*0.001);
		}
        }
	else {
                for (int i=0; i<3; i++){
		    ddcom(i) = 0;
		}
        }
*/
	yarp::sig::Vector & com_to_send = port_com->prepare();
        
	com_to_send.resize(3);
        for (int i=0; i<3; i++){
             com_to_send(i)=com(i);
        }
/*	for (int i=3; i<6; i++){
             com_to_send(i)=dcom(i-3);
        }
	for (int i=6; i<9; i++)
             com_to_send(i)=ddcom(i-6);

        old_com = com;
        old_dcom = dcom;
 */       
        //if (acc_counter>=2){
	port_com->write();
        //}
        //acc_counter++;
        
}
    


//============================
//-- xSens-Robot Joint Mapping
//============================
void retargetThread::getRobotJoints()
{
    //read joint angles value from xsens port
    Bottle *input = jointRead.read(); 

    /////////////////
    //  xSens to iCub
    if (robotName.find("icub") != std::string::npos){
	    double torso_pitch = (input->get(5).asDouble()+input->get(8).asDouble()+input->get(11).asDouble());
	    double torso_roll = (input->get(3).asDouble()+input->get(6).asDouble()+input->get(9).asDouble())*-1;
	    double torso_yaw = (input->get(4).asDouble()+input->get(7).asDouble()+input->get(10).asDouble())*-1; 
	    double neck_pitch = (input->get(17).asDouble())*-1;
	    double neck_roll = input->get(15).asDouble();
	    double neck_yaw = input->get(16).asDouble();
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
	

            jointPosCoM.resize(q.size());
            jointPosCoM << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;

            jointPos.resize(actuatedDOFs);
	    if ((joint_list.compare("ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << torso_pitch, torso_roll, torso_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
	    }
	    else if ((joint_list.compare("ROBOT_TORQUE_CONTROL_JOINTS") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << torso_pitch, torso_roll, torso_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
	    }
	    else if ((joint_list.compare("ROBOT_DYNAMIC_MODEL_JOINTS") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
	    }
	    else if ((joint_list.compare("ROBOT_DYNAMIC_MODEL_JOINTS") == 0) && (controller.compare("QP") == 0)){
		jointPos << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw;
 		for (int i=20; i<32; i++){
		    swap(m_minJointLimits(i),m_minJointLimits(i-14));
		    swap(m_maxJointLimits(i),m_maxJointLimits(i-14));
	        }
	        for (int i=20; i<32; i+=2){
		    swap(m_minJointLimits(i),m_minJointLimits(i-2));
		    swap(m_maxJointLimits(i),m_maxJointLimits(i-2));
		    swap(m_minJointLimits(i+1),m_minJointLimits(i-1));
		    swap(m_maxJointLimits(i+1),m_maxJointLimits(i-1));
 		}
	    }
	    else {
	        yError() << "[ERROR] this controller is not designed to control the selected joint list. Check the configuration file";
 	        return;
	    }
    }
    /////////////////////////////////////
    else {
	yError() << "[ERROR] xSens-robot joint mapping not defined (implementation of getRobotJoints() required!)";
    	return;
    }
	

    //convert to radiants
    if (jointFormat.compare("radians") == 0)
    {
	    for (int i = 0; i < jointPos.size(); i++){
		jointPos(i) = jointPos(i)*PI/180;
	    }
    }

    //convert to radiants
    for(int i =0; i < (q.size()); i++)
    {
	q(i) = jointPosCoM(i)*PI/180;
    }


   // Joint limits for all the joints (not only the actuated ones) !!!Fix me for also other joint_lists
    if (this->m_checkJointLimits && (joint_list.compare("ROBOT_DYNAMIC_MODEL_JOINTS") == 0)){
	    // Avoid joint limits
	    for (int i = 0; i < q.size(); i++){ 
	    	if (q(i) <= (m_minJointLimits(i) + offset))		//check min
			q(i) = m_minJointLimits(i) + offset;
		if (q(i) >= (m_maxJointLimits(i) - offset))		//check max
			q(i) = m_maxJointLimits(i) - offset;
	    }    
    }
    icub_model.setAng(q);

}


void retargetThread::avoidJointLimits() 
{
    for (int i = 0; i < jointPos.size(); i++){ 
    	if (jointPos(i) <= (m_minJointLimits(i) + offset))		//check min
		jointPos(i) = m_minJointLimits(i) + offset;
	if (jointPos(i) >= (m_maxJointLimits(i) - offset))		//check max
		jointPos(i) = m_maxJointLimits(i) - offset;
    } 	  
}


//------ RUN ------
void retargetThread::run()
{
    /*if( this->run_mutex_acquired )
    {
	yError() << "retarget: run_mutex already acquired at the beginning of run method.";
	yError() << "    this could cause some problems, please report an issue at https://github.com/robotology/codyco-modules/issues/new";
    }

    run_mutex.lock();
    this->run_mutex_acquired = true;
    */
    getRobotJoints();
    
    if (this->m_checkJointLimits && jointFormat.compare("radians") == 0)
    	avoidJointLimits();
       

    
    double    t = yarp::os::Time::now();
    mapping_run();
   
    //this->run_mutex_acquired = false;
    //run_mutex.unlock();
}


void retargetThread::mapping_run()
{
 
    publishJoints();

    //Compute com
    publishCom();
    
    acc_counter++;

    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

    if( printCountdown == 0 ) {
       
	double avgTime, stdDev, avgTimeUsed, stdDevUsed;

	getEstPeriod(avgTime, stdDev);
	getEstUsed(avgTimeUsed, stdDevUsed);

    }

}
   

void retargetThread::threadRelease()
{
    //run_mutex.lock();

    yInfo() << "Closing all the ports";
    jointRead.interrupt();
    jointRead.close();
    posRead.interrupt();
    posRead.close();
    port_joint.interrupt();
    port_joint.close();
    
    closePort(port_com);
    
    //run_mutex.unlock();
}

void retargetThread::closePort(yarp::os::Contactable *_port)
{
    if (_port)
    {
	_port->interrupt();
	_port->close();

	delete _port;
	_port = 0;
    }
}


