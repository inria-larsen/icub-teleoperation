/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

// Local includes
#include "retargetingThread.h"
#include <math.h>       

using namespace yarp::os;
using namespace std;
using namespace Eigen;
using namespace yarp::math;

const double PI = 3.141592653589793;


//===============================================
//        Retargeting Main THREAD
//===============================================

retargetingThread::retargetingThread(string _name,
			       string _robotName,
			       int _actuatedDOFs,
			       bool checkJointLimits,
                               Eigen::VectorXd minJointLimits,
                               Eigen::VectorXd maxJointLimits,
			       int _period,
			       double _offset,
                               Eigen::VectorXd ratioLimbs,
			       Eigen::VectorXd p_ref_T_h,
                               Eigen::VectorXd p_ref_T_r,
                               std::string _ref_frame,
                               std::string _start_pos)
    :  RateThread(_period),
       moduleName(_name),
       robotName(_robotName),
       actuatedDOFs(_actuatedDOFs),
       m_checkJointLimits(checkJointLimits),
       m_minJointLimits(minJointLimits),
       m_maxJointLimits(maxJointLimits),	
       offset(_offset),
       m_ratioLimbs(ratioLimbs),
       m_p_ref_T_h(p_ref_T_h),
       m_p_ref_T_r(p_ref_T_r),
       ref_frame(_ref_frame),
       start_pos(_start_pos),
       printCountdown(0),
       printPeriod(2000)
       
{

    yInfo() << "Launching retargetingThread with name : " << _name << " and robotName " << _robotName << " and period " << _period;
   
}


bool retargetingThread::threadInit()
{  
    if (this->m_checkJointLimits) {
        std::cout << "[INFO]Joint limits are:\nmin" <<m_minJointLimits.transpose() << "\nmax " << m_maxJointLimits.transpose() << "\n";
    } else {
        yInfo("Joint limits disabled");
    }
    
    //opening ports
    joint_port.open(string("/"+moduleName+"/q:o").c_str());
    pos_port.open(string("/"+moduleName+"/pos:o").c_str());
    //com_port.open(string("/"+moduleName+"/com:o").c_str());
  
    yInfo() << "retargetingThread::threadInit finished successfully.";

    return true;
}


void retargetingThread::publishJoints()
{
    Bottle& output = joint_port.prepare();
    output.clear();

    for (int i=0; i < jointPos.size(); i++){
        output.addDouble(jointPos(i));
    }

    joint_port.write();
}

void retargetingThread::publishPos()
{
    Bottle& output = pos_port.prepare();
    output.clear();
  
    for (int i=0; i < bodySegPos.size(); i++){
        output.addDouble(bodySegPos(i));
    }

    pos_port.write();
}


//----------------------------
//-- xSens-Robot Joint Retargeting
//----------------------------
void retargetingThread::getRobotJoints()
{
    //read joint positions from xsens
    Bottle *input = joint_port.read(false); 

    /////////////////
    //  xSens to iCub
    if (robotName.find("icub") != std::string::npos){
	if(input!=NULL){
	    streamingJoint = true;
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
	
	    jointPos.resize(actuatedDOFs);
    
	    jointPos << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
	}
         else {
            streamingJoint = false;
        }
    }
    /////////////////////////////////////
    else {
	yError() << "[ERROR] xSens-robot joint Retargeting not defined (implementation of getRobotJoints() required!)";
    	return;
    }
	

    //convert to radiants
    for (int i = 0; i < jointPos.size(); i++){
	jointPos(i) = jointPos(i)*PI/180;
    }
}


void retargetingThread::avoidJointLimits() 
{
    for (int i = 0; i < jointPos.size(); i++){ 
    	if (jointPos(i) <= (m_minJointLimits(i) + offset))		//check min
		jointPos(i) = m_minJointLimits(i) + offset;
	if (jointPos(i) >= (m_maxJointLimits(i) - offset))		//check max
		jointPos(i) = m_maxJointLimits(i) - offset;
    } 	  
}

//----------------------------
//-- xSens-Robot Body Segment Pose Retargeting
//----------------------------
void retargetingThread::getRobotPos()
{
    //read body segments pose from xsens
    //0:2 positions, 3:6 orientation(quaternions)
    Bottle *input = pos_port.read(false);
    if(input!=NULL){   
            streamingPos = true;
	    Eigen::VectorXd pelvis(7);
	    Eigen::VectorXd L5(7);
	    Eigen::VectorXd L3(7);
	    Eigen::VectorXd T12(7);
	    Eigen::VectorXd T8(7);
	    Eigen::VectorXd neck(7);
	    Eigen::VectorXd head(7);
	    Eigen::VectorXd r_shoulder(7);
	    Eigen::VectorXd r_upperArm(7);
	    Eigen::VectorXd r_forearm(7);
	    Eigen::VectorXd r_hand(7);
	    Eigen::VectorXd l_shoulder(7);
	    Eigen::VectorXd l_upperArm(7);
	    Eigen::VectorXd l_forearm(7);
	    Eigen::VectorXd l_hand(7);
	    Eigen::VectorXd r_upperLeg(7);
	    Eigen::VectorXd r_lowerLeg(7);
	    Eigen::VectorXd r_foot(7);
	    Eigen::VectorXd r_toe(7);
	    Eigen::VectorXd l_upperLeg(7);
	    Eigen::VectorXd l_lowerLeg(7);
	    Eigen::VectorXd l_foot(7);
	    Eigen::VectorXd l_toe(7);

	    for (int i=0; i<7; i++){
		pelvis(i) = input->get(i).asDouble();
		L5(i) = input->get(i+7).asDouble();
		L3(i) = input->get(i+14).asDouble();
		T12(i) = input->get(i+21).asDouble();
		T8(i) = input->get(i+28).asDouble();
		neck(i) = input->get(i+35).asDouble();
		head(i) = input->get(i+42).asDouble();
		r_shoulder(i) = input->get(i+49).asDouble();
		r_upperArm(i) = input->get(i+56).asDouble();
		r_forearm(i) = input->get(i+63).asDouble();
		r_hand(i) = input->get(i+70).asDouble();
		l_shoulder(i) = input->get(i+77).asDouble();
		l_upperArm(i) = input->get(i+84).asDouble();
		l_forearm(i) = input->get(i+91).asDouble();
		l_hand(i) = input->get(i+98).asDouble();
		r_upperLeg(i) = input->get(i+105).asDouble();
		r_lowerLeg(i) = input->get(i+112).asDouble();
		r_foot(i) = input->get(i+119).asDouble();
		r_toe(i) = input->get(i+126).asDouble();
		l_upperLeg(i) = input->get(i+133).asDouble();
		l_lowerLeg(i) = input->get(i+140).asDouble();
		l_foot(i) = input->get(i+147).asDouble();
		l_toe(i) = input->get(i+154).asDouble();
            }		
	    	
	    Eigen::Vector3d waist;
	    Eigen::Vector3d waist_head;
	    Eigen::Vector3d l_upperArm_hand;
	    Eigen::Vector3d r_upperArm_hand;
	    Eigen::Vector3d l_upperLeg_foot;
	    Eigen::Vector3d r_upperLeg_foot;

	    Eigen::Vector3d waist_headW;
	    Eigen::Vector3d l_upperArm_handW;
	    Eigen::Vector3d r_upperArm_handW;
	    Eigen::Vector3d l_upperLeg_footW;
	    Eigen::Vector3d r_upperLeg_footW;

	    // compute relative position in global frame
	    for (int i=0; i<3; i++) {
                waist(i) = (pelvis(i) + L5(i))/2;
		waist_head(i) = (head(i) - waist(i))*m_ratioLimbs(0);
		l_upperArm_hand(i) = (l_hand(i) - l_upperArm(i))*m_ratioLimbs(1);
		r_upperArm_hand(i) = (r_hand(i) - r_upperArm(i))*m_ratioLimbs(1);
		l_upperLeg_foot(i) = (l_foot(i) - l_upperLeg(i))*m_ratioLimbs(2); 
		r_upperLeg_foot(i) = (r_foot(i) - r_upperLeg(i))*m_ratioLimbs(2);    
	    }
	    
	   // rotation matrix from global frame to waist frame
            Eigen::Matrix3d Rwaist;
            Rwaist << 1-2*pelvis(5)*pelvis(5)-2*pelvis(6)*pelvis(6), 2*pelvis(4)*pelvis(5)-2*pelvis(3)*pelvis(6), 2*pelvis(4)*pelvis(6)+2*pelvis(3)*pelvis(5),
		      2*pelvis(4)*pelvis(5)+2*pelvis(3)*pelvis(6), 1-2*pelvis(4)*pelvis(4)-2*pelvis(6)*pelvis(6), 2*pelvis(5)*pelvis(6)-2*pelvis(3)*pelvis(4),
	              2*pelvis(4)*pelvis(6)-2*pelvis(3)*pelvis(5), 2*pelvis(5)*pelvis(6)+2*pelvis(3)*pelvis(4), 1-2*pelvis(4)*pelvis(4)-2*pelvis(5)*pelvis(5);

	    // rotate relative position in waist frame
	    for (int i=0; i<3; i++) {
		waist_headW(i) = Rwaist(i,0)*waist_head(0)+Rwaist(i,1)*waist_head(1)+Rwaist(i,2)*waist_head(2);
		l_upperArm_handW(i) = Rwaist(i,0)*l_upperArm_hand(0)+Rwaist(i,1)*l_upperArm_hand(1)+Rwaist(i,2)*l_upperArm_hand(2);
		r_upperArm_handW(i) = Rwaist(i,0)*r_upperArm_hand(0)+Rwaist(i,1)*r_upperArm_hand(1)+Rwaist(i,2)*r_upperArm_hand(2);
		l_upperLeg_footW(i) = Rwaist(i,0)*l_upperLeg_foot(0)+Rwaist(i,1)*l_upperLeg_foot(1)+Rwaist(i,2)*l_upperLeg_foot(2);
		r_upperLeg_footW(i) = Rwaist(i,0)*r_upperLeg_foot(0)+Rwaist(i,1)*r_upperLeg_foot(1)+Rwaist(i,2)*r_upperLeg_foot(2);
	    }
	 
	    bodySegPos.resize(15);
	    
            if (ref_frame.compare("waist")==0)
	    	bodySegPos << waist_headW, l_upperArm_handW, r_upperArm_handW, l_upperLeg_footW, r_upperLeg_footW;
	    else {
		if (start_pos.compare("T")==0){
                    Eigen::Vector3d waist_headTref;
	            Eigen::Vector3d l_upperArm_handTref;
	            Eigen::Vector3d r_upperArm_handTref;
	            Eigen::Vector3d l_upperLeg_footTref;
	            Eigen::Vector3d r_upperLeg_footTref;
		    
		    // Consider also the mapping of the initial T pose of the robot and the human 
		    for (int i=0; i<3; i++) {
			waist_headTref(i) = waist_head(i) + m_p_ref_T_r(i) - m_ratioLimbs(0)*m_p_ref_T_h(i);
			l_upperArm_handTref(i) = l_upperArm_hand(i) + m_p_ref_T_r(i+3) - m_ratioLimbs(1)*m_p_ref_T_h(i+3);
			r_upperArm_handTref(i) = r_upperArm_hand(i) + m_p_ref_T_r(i+6) - m_ratioLimbs(1)*m_p_ref_T_h(i+6);
			l_upperLeg_footTref(i) = l_upperLeg_foot(i) + m_p_ref_T_r(i+9) - m_ratioLimbs(2)*m_p_ref_T_h(i+9);
			r_upperLeg_footTref(i) = r_upperLeg_foot(i) + m_p_ref_T_r(i+12) - m_ratioLimbs(2)*m_p_ref_T_h(i+12);   
		    } 
		    bodySegPos << waist_headTref, l_upperArm_handTref, r_upperArm_handTref, l_upperLeg_footTref, r_upperLeg_footTref;
		}
                else{
		    bodySegPos << waist_head, l_upperArm_hand, r_upperArm_hand, l_upperLeg_foot, r_upperLeg_foot;
		}
	    }
    }
     else {
            streamingPos = false;  
    }

}


//------ RUN ------
void retargetingThread::run()
{
    getRobotJoints();
    getRobotPos();
    
    if (this->m_checkJointLimits)
    	avoidJointLimits();
       
    double    t = yarp::os::Time::now();
    mapping_run();
}


void retargetingThread::mapping_run()
{
    if (streamingJoint){
        publishJoints();
    }
    if (streamingPos){
        publishPos();
    }
    
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

    if( printCountdown == 0 ) {
       
	double avgTime, stdDev, avgTimeUsed, stdDevUsed;

	getEstPeriod(avgTime, stdDev);
	getEstUsed(avgTimeUsed, stdDevUsed);

    }
}
   

void retargetingThread::closePort()
{
    yInfo() << "Closing joint port";
    joint_port.interrupt();
    joint_port.close();
    yInfo() << "joint port closed";
    yInfo() << "Closing position port";
    pos_port.interrupt();
    pos_port.close();
    yInfo() << "position port closed";
}


