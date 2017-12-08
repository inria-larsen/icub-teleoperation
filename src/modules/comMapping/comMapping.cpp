// YARP headers
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/PortInfo.h>

// iDynTree headers
#include <iCub/iDynTree/DynTree.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <urdf_exception/exception.h>

#include <string>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace yarp::os;
using namespace yarp::sig;

const double PI = 3.141592653589793;

//===============================================
//        ComMapping Main THREAD (runs every 10ms)
//===============================================

class comMappingThread: public RateThread
{
protected:

    string name; /**<Name used for the ports*/

    BufferedPort<Bottle> port; /**<Port that reads the joint status and outputs the com coordinates*/
   
    unsigned int status_ = STATUS_STOPPED;

    int count = 0;

public:

    comMappingThread(const double _period, string m_name, int status, string m_robot, string urdf, int nDOFs, string refFrame;)
        :
          RateThread(int(_period*1000.0)),
          name(m_name),
          status_(status),
	  m_robot(robot),
          m_urdf(urdf),
	  m_nDOFs(nDOFs),
	  m_refFrame(refFrame)
	  

    {
        yInfo("comMapping: thread created");

    }

    void startCaptureData(int status){
        xsens.startDataStream();
        status_ = status;

    }
    void stopCaptureData(int status){
        xsens.stopDataStream();
        status_ = status;
    }

    virtual bool threadInit()
    {
	
        std::string urdf_filename = rf.findFile(urdf); 

        yInfo() << "Trying to open " << urdf_filename << " as robot model";

        iCub::iDynTree::DynTree robot_model;

        bool ok = robot_model.loadURDFModel(urdf_filename);

        if( !ok )
        {
	   std::cerr << "Loading urdf file " << urdf_filename << " failed, exiting" << std::endl;
	   return EXIT_FAILURE;
        }

        //opening ports
        port.open(string("/"+name+"/com:i").c_str());
        
        return true;

    }

    virtual void threadRelease()
    {
        //closing all the ports
        
        port.interrupt();
        port.close();

        yInfo("comMapping: thread closing");

    }

    //------ RUN -------
    virtual void run()
    {
        count++;
        // cyclic operations should be put here!
        
	//read joint angles value from xsens port
        Bottle *input = port.read();  //to move to getData()
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

	VectorXd jointInput(32);
	jointInput << neck_pitch*PI/180, neck_roll*PI/180, neck_yaw*PI/180, torso_yaw*PI/180, torso_roll*PI/180, torso_pitch*PI/180, l_shoulder_pitch*PI/180, l_shoulder_roll*PI/180, l_shoulder_yaw*PI/180, l_elbow*PI/180, l_wrist_prosup*PI/180, l_wrist_pitch*PI/180, l_wrist_yaw*PI/180, r_shoulder_pitch*PI/180, r_shoulder_roll*PI/180, r_shoulder_yaw*PI/180, r_elbow*PI/180, r_wrist_prosup*PI/180, r_wrist_pitch*PI/180, r_wrist_yaw*PI/180, l_hip_pitch*PI/180, l_hip_roll*PI/180, l_hip_yaw*PI/180, l_knee*PI/180, l_ankle_pitch*PI/180, l_ankle_roll*PI/180, r_hip_pitch*PI/180, r_hip_roll*PI/180, r_hip_yaw*PI/180, r_knee*PI/180, r_ankle_pitch*PI/180, r_ankle_roll*PI/180;

	yarp::sig::Vector q(nDOFs);
	for(int i =0; i < 32); i++)
	{
		q(i) = jointInput(i);
	}
	
	for(int i =32; i < q.size(); i++)
	{
		q(i) = 0;
	}
	
        // Set the joint values
	robot_model.setAng(q);
	
	//Get the CoM coordinates w.r.t to the left foot frame 
	int index = robot_model.getLinkIndex(refFrame); // l_foot
	yarp::sig::Vector com(3);
	com = robot_model.getCOM(index);

	//KDLtoYarp(comk,com);
        //double comi = com.operator()(0);
        //double comii = com.operator()(1);
        //double comiii = com.operator()(2);	

        //send the data
	Bottle& output = port.prepare();
	output.clear();
        
	output.addDouble(com(0));
	output.addDouble(com(1));
	output.addDouble(com(2));
	/*CoM velocity set to 0 if not specified*/
	output.addDouble(0.0);
	output.addDouble(0.0);
	output.addDouble(0.0);
	/*CoM acceleration set to 0 if not specified*/
	output.addDouble(0.0);
	output.addDouble(0.0);
	output.addDouble(0.0);  

	port.write();
        }
        
}



    


//---------------------------------------------------------
//                  MAIN
//---------------------------------------------------------
int main(int argc, char * argv[])
{
   
    ResourceFinder rf;
    rf.setDefaultContext("icub-teleoperation");
    rf.setDefaultConfigFile("comMapping.ini");
    rf.configure(argc,argv);
  
    if (rf.check("help"))
    {
		printf("\n");
		yInfo("[comMapping] Options:");
        yInfo("		--urdf             file: name of the URDF file containing the model of the robot.");
        yInfo("		--from             from: of the file .ini user for configuration." );
	yInfo("  	--name             name:   the name of the module (default comMapping).");
	yInfo("  	--robot            robot:  the name of the robot (default icubSim).");
        printf("\n");

        return 0;
    }
    
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return -1;
    }

    comMapping module;
    module.runModule(rf);

    return 0;
}


