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
			       double _offset)
    :  RateThread(_period),
       moduleName(_name),
       robotName(_robotName),
       actuatedDOFs(_actuatedDOFs),
       m_checkJointLimits(checkJointLimits),
       m_minJointLimits(minJointLimits),
       m_maxJointLimits(maxJointLimits),	
       offset(_offset),
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
    pos_port.open(string("/"+moduleName+"/p:o").c_str());
  
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
    Bottle *input = joint_port.read(); 

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
	
	    bodySegPos.resize(actuatedDOFs);
    
	    jointPos << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
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
//-- xSens-Robot Body Segment Position Retargeting
//----------------------------
void retargetingThread::getRobotPos()
{
    //read body segments positions from xsens
    Bottle *input = pos_port.read(); 

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
	
	    jointPos.resize(actuatedDOFs);
    
	    jointPos << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
    }
    /////////////////////////////////////
    else {
	yError() << "[ERROR] xSens-robot body segment position Retargeting not defined (implementation of getRobotPos() required!)";
    	return;
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
    publishJoints();
    publishPos();
    
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


