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
    frame_counter = 0;
    counter = 0;
    n_pose = false;

   
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
    //start if in n-pose
    safe_start =yarp_options.find("safe_start").asBool();
    //tolerance range for the safe start
    minJointSpan_yarp =yarp_options.find("minJointSpan").asList();
    maxJointSpan_yarp =yarp_options.find("maxJointSpan").asList();
    
    minJointSpan.resize(minJointSpan_yarp->size());
    maxJointSpan.resize(maxJointSpan_yarp->size());    
    for (int i=0; i<minJointSpan.size(); i++){
        minJointSpan(i) = minJointSpan_yarp->get(i).asDouble();
        maxJointSpan(i) = maxJointSpan_yarp->get(i).asDouble();
    }
 
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

    output.addDouble(frame_counter);

    for (int i=0; i < jointPos.size(); i++){
        output.addDouble(jointPos(i));
    }
    
    port_joint.write();
}

void retargetThread::publishCom()
{
	// Stream com in index frame
        yarp::sig::Vector com = icub_model.getCOM(link_index);
       
	yarp::sig::Vector & com_to_send = port_com->prepare();
        
	com_to_send.resize(3);
        for (int i=0; i<3; i++){
             com_to_send(i)=com(i);
        }

	port_com->write();
        
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
	    double torso_pitch = (input->get(5).asDouble()+input->get(8).asDouble()+input->get(11).asDouble()) - offset_torso(0);
	    double torso_roll = ((input->get(3).asDouble()+input->get(6).asDouble()+input->get(9).asDouble())*-1) - offset_torso(1);
	    double torso_yaw = ((input->get(4).asDouble()+input->get(7).asDouble()+input->get(10).asDouble())*-1) - offset_torso(2); 
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
	    double l_hip_pitch = (input->get(56).asDouble()) - offset_l_leg(0);
	    double l_hip_roll = (input->get(54).asDouble()) - offset_l_leg(1);
	    double l_hip_yaw = ((input->get(55).asDouble())*-1) - offset_l_leg(2);
	    double l_knee = ((input->get(59).asDouble())*-1) - offset_l_leg(3);
	    double l_ankle_pitch = ((input->get(62).asDouble())*-1) - offset_l_leg(4);
	    double l_ankle_roll = (input->get(60).asDouble()) - offset_l_leg(5);
	    double r_hip_pitch = (input->get(44).asDouble()) - offset_r_leg(0);
	    double r_hip_roll = (input->get(42).asDouble()) - offset_r_leg(1);
	    double r_hip_yaw = ((input->get(43).asDouble())*-1) - offset_r_leg(2);
	    double r_knee = ((input->get(47).asDouble())*-1) - offset_r_leg(3);
	    double r_ankle_pitch = ((input->get(50).asDouble())*-1) - offset_r_leg(4);
            double r_ankle_roll = (input->get(48).asDouble()) - offset_r_leg(5);
 
 
            jointPosCoM.resize(q.size());
            jointPosCoM << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;

            jointPos.resize(actuatedDOFs);
	    if ((joint_list.compare("ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << torso_pitch, torso_roll, torso_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
	    }
	    else if ((joint_list.compare("ROBOT_TORSO_JOINTS") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << torso_pitch,torso_roll,torso_yaw;
	    }
            else if ((joint_list.compare("ROBOT_LEFT_LEG_JOINTS") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,l_ankle_pitch,l_ankle_roll;
	    }
            else if ((joint_list.compare("ROBOT_RIGHT_LEG_JOINTS") == 0) && (controller.compare("TorqueBalancing") == 0)){
		jointPos << r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll;
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

            //---------------------------
	    //-- Safe start: Check n-pose
            //---------------------------
            if (safe_start) {        
                if((jointPos(0)>maxJointSpan(0)) || (jointPos(0)<minJointSpan(0)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] TORSO PITCH: " << jointPos(0) << "\nn-pose range: " << minJointSpan(0) << " - " << maxJointSpan(0)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = true;
		}
                if((jointPos(1)>maxJointSpan(1)) || (jointPos(1)<minJointSpan(1)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] TORSO ROLL: " << jointPos(1) << "\nn-pose range: " << minJointSpan(1) << " - " << maxJointSpan(1)<< "\n";
		   n_pose = false;
		}else{
		    n_pose = n_pose && true;
		}
                if((jointPos(2)>maxJointSpan(2)) || (jointPos(2)<minJointSpan(2)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] TORSO YAW: " << jointPos(2) << "\nn-pose range: " << minJointSpan(2) << " - " << maxJointSpan(2)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(3)>maxJointSpan(3)) || (jointPos(3)<minJointSpan(3)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] NECK PITCH: " << jointPos(3) << "\nn-pose range: " << minJointSpan(3) << " - " << maxJointSpan(3)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(4)>maxJointSpan(4)) || (jointPos(4)<minJointSpan(4)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] NECK ROLL: " << jointPos(4) << "\nn-pose range: " << minJointSpan(4) << " - " << maxJointSpan(4)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(5)>maxJointSpan(5)) || (jointPos(5)<minJointSpan(5)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] NECK YAW: " << jointPos(5) << "\nn-pose range: " << minJointSpan(5) << " - " << maxJointSpan(5)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(6)>maxJointSpan(6)) || (jointPos(6)<minJointSpan(6)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L SHOULDER PITCH: " << jointPos(6) << "\nn-pose range: " << minJointSpan(6) << " - " << maxJointSpan(6)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(7)>maxJointSpan(7)) || (jointPos(7)<minJointSpan(7)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L SHOULDER ROLL: " << jointPos(7) << "\nn-pose range: " << minJointSpan(7) << " - " << maxJointSpan(7)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(8)>maxJointSpan(8)) || (jointPos(8)<minJointSpan(8)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L SHOULDER YAW: " << jointPos(8) << "\nn-pose range: " << minJointSpan(8) << " - " << maxJointSpan(8)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(9)>maxJointSpan(9)) || (jointPos(9)<minJointSpan(9)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L ELBOW: " << jointPos(9) << "\nn-pose range: " << minJointSpan(9) << " - " << maxJointSpan(9)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}		
                if((jointPos(10)>maxJointSpan(10)) || (jointPos(10)<minJointSpan(10)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L WRIST PROSUP: " << jointPos(10) << "\nn-pose range: " << minJointSpan(10) << " - " << maxJointSpan(10)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(11)>maxJointSpan(11)) || (jointPos(11)<minJointSpan(11)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L WRIST PITCH: " << jointPos(11) << "\nn-pose range: " << minJointSpan(11) << " - " << maxJointSpan(11)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(12)>maxJointSpan(12)) || (jointPos(12)<minJointSpan(12)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L WRIST YAW: " << jointPos(12) << "\nn-pose range: " << minJointSpan(12) << " - " << maxJointSpan(12)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(13)>maxJointSpan(13)) || (jointPos(13)<minJointSpan(13)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R SHOULDER PITCH: " << jointPos(13) << "\nn-pose range: " << minJointSpan(13) << " - " << maxJointSpan(13)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(14)>maxJointSpan(14)) || (jointPos(14)<minJointSpan(14)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R SHOULDER ROLL: " << jointPos(14) << "\nn-pose range" << minJointSpan(14) << " - " << maxJointSpan(14)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(15)>maxJointSpan(15)) || (jointPos(15)<minJointSpan(15)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R SHOULDER YAW: " << jointPos(15) << "\nn-pose range: " << minJointSpan(15) << " - " << maxJointSpan(15)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(16)>maxJointSpan(16)) || (jointPos(16)<minJointSpan(16)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R ELBOW: " << jointPos(16) << "\nn-pose range: " << minJointSpan(16) << " - " << maxJointSpan(16)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(17)>maxJointSpan(17)) || (jointPos(17)<minJointSpan(17)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R WRIST PROSUP: " << jointPos(17) << "\nn-pose range: " << minJointSpan(17) << " - " << maxJointSpan(17)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(18)>maxJointSpan(18)) || (jointPos(18)<minJointSpan(18)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R WRIST PITCH gap: " << jointPos(18) << "\nn-pose range: " << minJointSpan(18) << " - " << maxJointSpan(18)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(19)>maxJointSpan(19)) || (jointPos(19)<minJointSpan(19)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R WRIST YAW: " << jointPos(19) << "\nn-pose range: " << minJointSpan(19) << " - " << maxJointSpan(19)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(20)>maxJointSpan(20)) || (jointPos(20)<minJointSpan(20)))
		{
                   if (counter%100==0)
		  	 std::cout << "[INFO] L HIP PITCH: " << jointPos(20) << "\nn-pose range: " << minJointSpan(20) << " - " << maxJointSpan(20)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(21)>maxJointSpan(21)) || (jointPos(21)<minJointSpan(21)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L HIP ROLL: " << jointPos(21) << "\nn-pose range: " << minJointSpan(21) << " - " << maxJointSpan(21)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(22)>maxJointSpan(22)) || (jointPos(22)<minJointSpan(22)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L HIP YAW: " << jointPos(22) << "\nn-pose range: " << minJointSpan(22) << " - " << maxJointSpan(22)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(23)>maxJointSpan(23)) || (jointPos(23)<minJointSpan(23)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L KNEE: " << jointPos(23) << "\nn-pose range: " << minJointSpan(23) << " - " << maxJointSpan(23)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(24)>maxJointSpan(24)) || (jointPos(24)<minJointSpan(24)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L ANKLE PITCH: " << jointPos(24) << "\nn-pose range: " << minJointSpan(24) << " - " << maxJointSpan(24)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(25)>maxJointSpan(25)) || (jointPos(25)<minJointSpan(25)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] L ANKLE ROLL: " << jointPos(25) << "\nn-pose range: " << minJointSpan(25) << " - " << maxJointSpan(25)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(26)>maxJointSpan(26)) || (jointPos(26)<minJointSpan(26)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R HIP PITCH: " << jointPos(26) << "\nn-pose range: " << minJointSpan(26) << " - " << maxJointSpan(26)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(27)>maxJointSpan(27)) || (jointPos(27)<minJointSpan(27)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R HIP ROLL: " << jointPos(27) << "\nn-pose range: " << minJointSpan(27) << " - " << maxJointSpan(27)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(28)>maxJointSpan(28)) || (jointPos(28)<minJointSpan(28)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R HIP YAW: " << jointPos(28) << "\nn-pose range: " << minJointSpan(28) << " - " << maxJointSpan(28)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(29)>maxJointSpan(29)) || (jointPos(29)<minJointSpan(29)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R KNEE: " << jointPos(29) << "\nn-pose range: " << minJointSpan(29) << " - " << maxJointSpan(29)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(30)>maxJointSpan(30)) || (jointPos(30)<minJointSpan(30)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R ANKLE PITCH: " << jointPos(30) << "\nn-pose range: " << minJointSpan(30) << " - " << maxJointSpan(30)<< "\n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
                if((jointPos(31)>maxJointSpan(31)) || (jointPos(31)<minJointSpan(31)))
		{
                   if (counter%100==0)
		   	std::cout << "[INFO] R ANKLE ROLL: " << jointPos(31) << "\nn-pose range: " << minJointSpan(31) << " - " << maxJointSpan(31)<< "\n \n \n \n";
		   n_pose = false;
		}else{
		   n_pose = n_pose && true;
		}
		if(n_pose){
		   safe_start = false;
		}

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
    
    if (!safe_start)
       frame_counter++;

    counter++;

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
    jointRead.close();
    posRead.close();
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

