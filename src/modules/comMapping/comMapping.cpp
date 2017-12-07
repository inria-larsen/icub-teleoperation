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

/*****************************************************************/
int main(int argc, char *argv[])
{
    // Use yarp to parse the configuration parameters
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","comMapping");
    rf.setDefaultConfigFile("config.ini");         //default config file name.
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:\n\n";
        yInfo() << "\t--urdf    file: name of the URDF file containing"
                   " the model of the robot.  \n";
        yInfo()<< "\t--from             :Name of the file .ini user for configuration." ;
        yInfo()<< "\t--wbi_conf_file    :Name of the configuration file used for yarpWholeBodyInterface ."
                                           ;
        yInfo()<< "\t--rate             :Period (in ms) used by the module. Default set to 10ms."                                                      ;
        yInfo()<< "\t--name             :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyDynamicsTree."    ;
        yInfo()<< "\t--assume_fixed     :Use a link as a kinematic root in estimation and calibration (assuming a constant gravity). Possible options: (root_link, l_foot_dh_frame, r_foot_dh_frame).";
        yInfo()<< "\t--assume_fixed_from_odometry :Use the fixed link from odometry for assume a constant gravity in estimation and calibration";
        yInfo()<< "                                 ***NOTE: with this option only the calibration on two feet is supported. Furthermore the link should not be switched during calibration.";
        yInfo()<< "                                 Furthermore the only supported fixed link for odometry are r_foot and l_foot***";
        yInfo()<< "\t--smooth_calibration switch_period : Perform a smooth calibration (i.e.: don't stop estimating torques during calibration, and then smoothly change the ft offsets)";
        yInfo()<< "\t                                     the switch_period express the period (in ms) used for offset interpolation.";
        return 0;
    }

    std::string urdf_filename = rf.findFile("icub_for_gazebo.urdf");  //icub.urdf

    yInfo() << "Trying to open " << urdf_filename << " as robot model";
    iCub::iDynTree::DynTree robot_model;
    bool ok = robot_model.loadURDFModel(urdf_filename);

    if( !ok )
    {
        std::cerr << "Loading urdf file " << urdf_filename << " failed, exiting" << std::endl;
        return EXIT_FAILURE;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

//-----------------------------------------------
    BufferedPort<Bottle> port;
    port.open("/xsensToRobot/com");
    while (true) {
        cout << "waiting for input" << endl;
        Bottle *input = port.read();
        if (input!=NULL) {
		cout << "got " << input->toString().c_str() << endl;
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
		
		yarp::sig::Vector q(38);
		for(int i =0; i < (q.size()-6); i++)
		{
		q(i) = jointInput(i);
		}
		for(int i =(q.size()-6); i < q.size(); i++)
		{
		q(i) = 0;
		}
		// Set the joint values
		robot_model.setAng(q);
		
		//Get the CoM coordinates w.r.t to the left foot frame
		int index = robot_model.getLinkIndex("l_foot");
		 yarp::sig::Vector com(3);
 		com = robot_model.getCOM(index);
                
		Bottle& output = port.prepare();
		output.clear();
	        
               
		//KDLtoYarp(comk,com);
               // double comi = com.operator()(0);
                //double comii = com.operator()(1);
	      //  double comiii = com.operator()(2);	
 
		
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
    return 0;
}

