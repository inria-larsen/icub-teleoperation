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
			       Eigen::VectorXd j_start_r_T_,
                   Eigen::VectorXd p_start_r_T_,
                   double base_start_r_T_,
                   std::string _ref_frame,
                   std::string _start_pos,
                   bool _stream_feet,
                   bool _stream_base,
                   std::string _joint_value,
	               Eigen::VectorXd qstart_r_,
	               std::string urdf)
:   RateThread(_period),
	moduleName(_name),
	robotName(_robotName),
	actuatedDOFs(_actuatedDOFs),
	m_checkJointLimits(checkJointLimits),
	m_minJointLimits(minJointLimits),
	m_maxJointLimits(maxJointLimits),	
	offset(_offset),
	m_ratioLimbs(ratioLimbs),
	j_start_r_T(j_start_r_T_),
	p_start_r_T(p_start_r_T_),
	base_start_r_T(base_start_r_T_),
	ref_frame(_ref_frame),
	start_pos(_start_pos),
	stream_feet(_stream_feet),
	stream_base(_stream_base),
	joint_value(_joint_value),
	qstart_r(qstart_r_),
	urdf_file_path(urdf),
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

	bool ok = icub_model.loadURDFModel(urdf_file_path);

    if(ok){
        yInfo("Dummy model of the robot successfully istantiated");
    }  
    else {
    	std::cerr << "Loading urdf file failed, exiting" << std::endl;
        return false;
    }


    icub_model.setAng(eigenToYarp(qstart_r));
    std::string link_name = "l_foot";
    link_index = icub_model.getLinkIndex(link_name);
    dummyCom_start = icub_model.getCOM(link_index);

	//opening ports
	joint_port.open(string("/"+moduleName+"/q:o").c_str());
	pos_port.open(string("/"+moduleName+"/pos:o").c_str());
	com_port.open(string("/"+moduleName+"/com:o").c_str());

	com.resize(5); //x,y,z + offset from left foot + deltaCoMx
	jointPos.resize(actuatedDOFs);
	bodySegPos.resize(15);
	j_start_h.resize(actuatedDOFs);
	p_start_h.resize(15);

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
	if (stream_feet){
		for (int i=0; i < 3; i++){
			output.addDouble(l_foot_yaw(i));
		}
		for (int i=0; i < 3; i++){
			output.addDouble(r_foot_yaw(i));
		}
	}
	if (stream_base){
		output.addDouble(basei_r);
	}
	output.addDouble(n_delta_roll);
	output.addDouble(n_delta_pitch);
	output.addDouble(n_delta_yaw);

	pos_port.write();
}

void retargetingThread::publishCoM()
{
	Bottle& output = com_port.prepare();
	output.clear();

	for (int i=0; i < com.size(); i++){
		output.addDouble(com(i));
	}

	com_port.write();
}


//----------------------------
//-- xSens-Robot Joint Retargeting
//----------------------------
void retargetingThread::getRobotJoints()
{
	//read joint positions from xsens
	Bottle *input = joint_port.read(false); 

	//++++++++ xSens to iCub +++++++++++
	if (robotName.find("icub") != std::string::npos){
		if(input!=NULL){
			streamingJoint = true;

			// Robot and human joint angle vectors
			Eigen::VectorXd ji_h(actuatedDOFs);
			Eigen::VectorXd ji_r(actuatedDOFs);
			Eigen::VectorXd delta_j_r(actuatedDOFs);
			Eigen::VectorXd delta_j_h(actuatedDOFs);

			double torso_pitch = (input->get(5).asDouble()+input->get(8).asDouble()+input->get(11).asDouble());
			double torso_roll = (input->get(3).asDouble()+input->get(6).asDouble()+input->get(9).asDouble())*-1;
			double torso_yaw = (input->get(1).asDouble()+input->get(4).asDouble()+input->get(7).asDouble()+input->get(10).asDouble()+input->get(13).asDouble())*-1; 
			double neck_pitch = (input->get(17).asDouble())*-1;
			double neck_roll = input->get(15).asDouble();
			double neck_yaw = input->get(16).asDouble();
			double l_shoulder_pitch = (input->get(35).asDouble())*-1;
			double l_shoulder_roll = input->get(33).asDouble();
			double l_shoulder_yaw = input->get(34).asDouble();
			double l_elbow = input->get(38).asDouble();
			double l_wrist_prosup = (input->get(40).asDouble());
			double l_wrist_pitch = (input->get(39).asDouble())*-1;
			double l_wrist_yaw = (input->get(41).asDouble())*-1;
			double r_shoulder_pitch = (input->get(23).asDouble())*-1;
			double r_shoulder_roll = input->get(21).asDouble();
			double r_shoulder_yaw = input->get(22).asDouble();
			double r_elbow = input->get(26).asDouble();
			double r_wrist_prosup = (input->get(28).asDouble());
			double r_wrist_pitch = (input->get(27).asDouble())*-1;
			double r_wrist_yaw = (input->get(29).asDouble())*-1;
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

			ji_h << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;

			//convert to radiants
			for (int i = 0; i < ji_h.size(); i++){
				ji_h(i) = ji_h(i)*PI/180;
			}

			// store starting reference human joint angles
			if(firstRunJ) {
				j_start_h = ji_h;
			}
			firstRunJ = false;

			//compute delta joint angles
			delta_j_h = ji_h - j_start_h;
			delta_j_r = delta_j_h;
			

			if (joint_value.compare("istantaneous")==0){
				jointPos = ji_h;
			}
			if(joint_value.compare("delta")==0){
				// ji_r = j_start_r + delta_j_r 
				//		= j_start_r + delta_j_h
				//		= j_start_r + (ji_h - j_start_h)
				if (start_pos.compare("T")==0){
				    ji_r = j_start_r_T + delta_j_r;
		
					jointPos = ji_r;
				}
				else{ // delta_j_r
					jointPos = delta_j_r;
				}
			}
			else{ //hybrid
				jointPos = delta_j_r; 
				// istantaneous for the wrists
				for (int i=10; i<13; i++){
					jointPos(i) = ji_h(i);
					jointPos(i+7) = ji_h(i+7);
				}
			}
		}
		else {
			streamingJoint = false;
		}
	}
	//++++++++++++++++++++++++++++
	else {
		yError() << "[ERROR] xSens-robot joint Retargeting not defined (implementation of getRobotJoints() required!)";
		return;
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
		r_foot.resize(7);
		Eigen::VectorXd r_toe(7);
		Eigen::VectorXd l_upperLeg(7);
		Eigen::VectorXd l_lowerLeg(7);
		l_foot.resize(7);
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

		// relative positions in global frame
		Eigen::Vector3d waist;
		Eigen::Vector3d waist_head;
		Eigen::Vector3d l_upperArm_hand;
		Eigen::Vector3d r_upperArm_hand;
		Eigen::Vector3d l_upperLeg_foot;
		Eigen::Vector3d r_upperLeg_foot;
		// relative positions in waist frame
		Eigen::Vector3d waist_headW;
		Eigen::Vector3d l_upperArm_handW;
		Eigen::Vector3d r_upperArm_handW;
		Eigen::Vector3d l_upperLeg_footW;
		Eigen::Vector3d r_upperLeg_footW;

		// Robot and human position vectors
		Eigen::VectorXd pi_h(15);
		Eigen::VectorXd pi_r(15);
		Eigen::VectorXd delta_p_r(15);
		Eigen::VectorXd delta_p_h(15);

		// Robot and human base z position
		double basei_h;
		double delta_base_r;
		double delta_base_h;

		// Robot and human base orientation
		Eigen::VectorXd	base_quat_h(4);
		double roll_h;
		double pitch_h;
		double yaw_h;

		// Robot and human neck orientation
		Eigen::VectorXd	neck_quat_h(4);
		double n_roll_h;
		double n_pitch_h;
		double n_yaw_h;

		// Robot and human feet orientation
		Eigen::VectorXd	lf_quat_h(4);
		double lf_roll_h;
		double lf_pitch_h;
		double lf_yaw_h;
		Eigen::VectorXd	rf_quat_h(4);
		double rf_roll_h;
		double rf_pitch_h;
		double rf_yaw_h;



		// compute human relative position in global frame
		for (int i=0; i<3; i++) {
			waist(i) = (pelvis(i) + L5(i))/2;
			waist_head(i) = head(i) - waist(i);
			l_upperArm_hand(i) = l_hand(i) - l_upperArm(i);
			r_upperArm_hand(i) = r_hand(i) - r_upperArm(i);
			l_upperLeg_foot(i) = l_foot(i) - l_upperLeg(i); 
			r_upperLeg_foot(i) = r_foot(i) - r_upperLeg(i);    
		}

		if (ref_frame.compare("waist")==0) { //relative positions in waist frame
			// rotation matrix from global frame to waist frame
			Eigen::Matrix3d Rwaist;
			Rwaist << 1-2*pelvis(5)*pelvis(5)-2*pelvis(6)*pelvis(6), 2*pelvis(4)*pelvis(5)-2*pelvis(3)*pelvis(6), 2*pelvis(4)*pelvis(6)+2*pelvis(3)*pelvis(5),
			2*pelvis(4)*pelvis(5)+2*pelvis(3)*pelvis(6), 1-2*pelvis(4)*pelvis(4)-2*pelvis(6)*pelvis(6), 2*pelvis(5)*pelvis(6)-2*pelvis(3)*pelvis(4),
			2*pelvis(4)*pelvis(6)-2*pelvis(3)*pelvis(5), 2*pelvis(5)*pelvis(6)+2*pelvis(3)*pelvis(4), 1-2*pelvis(4)*pelvis(4)-2*pelvis(5)*pelvis(5);

			// rotate human relative position in waist frame
			for (int i=0; i<3; i++) {
				waist_headW(i) = Rwaist(i,0)*waist_head(0)+Rwaist(i,1)*waist_head(1)+Rwaist(i,2)*waist_head(2);
				l_upperArm_handW(i) = Rwaist(i,0)*l_upperArm_hand(0)+Rwaist(i,1)*l_upperArm_hand(1)+Rwaist(i,2)*l_upperArm_hand(2);
				r_upperArm_handW(i) = Rwaist(i,0)*r_upperArm_hand(0)+Rwaist(i,1)*r_upperArm_hand(1)+Rwaist(i,2)*r_upperArm_hand(2);
				l_upperLeg_footW(i) = Rwaist(i,0)*l_upperLeg_foot(0)+Rwaist(i,1)*l_upperLeg_foot(1)+Rwaist(i,2)*l_upperLeg_foot(2);
				r_upperLeg_footW(i) = Rwaist(i,0)*r_upperLeg_foot(0)+Rwaist(i,1)*r_upperLeg_foot(1)+Rwaist(i,2)*r_upperLeg_foot(2);
			}
			pi_h << waist_headW, l_upperArm_handW, r_upperArm_handW, l_upperLeg_footW, r_upperLeg_footW;
		}
		else{
			pi_h << waist_head, l_upperArm_hand, r_upperArm_hand, l_upperLeg_foot, r_upperLeg_foot;
		}

		basei_h = pelvis(2);
		base_quat_h << pelvis(3), pelvis(4), pelvis(5), pelvis(6);
		roll_h = toRoll(base_quat_h);
		pitch_h = toPitch(base_quat_h);
		yaw_h = toYaw(base_quat_h);

		neck_quat_h << neck(3), neck(4), neck(5), neck(6);
		n_roll_h = toRoll(neck_quat_h);
		n_pitch_h = toPitch(neck_quat_h);
		n_yaw_h = toYaw(neck_quat_h);

		lf_quat_h << l_foot(3), l_foot(4), l_foot(5), l_foot(6);
		lf_roll_h = toRoll(lf_quat_h);
		lf_pitch_h = toPitch(lf_quat_h);
		lf_yaw_h = toYaw(lf_quat_h);

		rf_quat_h << r_foot(3), r_foot(4), r_foot(5), r_foot(6);
		rf_roll_h = toRoll(rf_quat_h);
		rf_pitch_h = toPitch(rf_quat_h);
		rf_yaw_h = toYaw(rf_quat_h);

		Eigen::Matrix3d Ryaw_lf;
		Eigen::Matrix3d Ryaw_rf;

		Ryaw_lf << cos(lf_yaw_h), -1*sin(lf_yaw_h), 0,
				sin(lf_yaw_h), cos(lf_yaw_h), 0,
				0, 0, 1;
		Ryaw_rf << cos(rf_yaw_h), -1*sin(rf_yaw_h), 0,
				sin(rf_yaw_h), cos(rf_yaw_h), 0,
				0, 0, 1;

		l_footi << l_foot(0), l_foot(1), l_foot(2);
		r_footi << r_foot(0), r_foot(1), r_foot(2);

		for (int i=0; i<3; i++) {
			l_foot_yaw(i) = Ryaw_lf(i,0)*l_footi(0)+Ryaw_lf(i,1)*l_footi(1)+Ryaw_lf(i,2)*l_footi(2);
			r_foot_yaw(i) = Ryaw_rf(i,0)*r_footi(0)+Ryaw_rf(i,1)*r_footi(1)+Ryaw_rf(i,2)*r_footi(2);
		}
		
		// store starting reference human position
		if(firstRunPos) {
			p_start_h = pi_h;

			base_start_h = basei_h;

			roll_start_h = roll_h;
			pitch_start_h = pitch_h;
			yaw_start_h = yaw_h;

			n_roll_start_h = n_roll_h;
			n_pitch_start_h = n_pitch_h;
			n_yaw_start_h = n_yaw_h;

			lf_roll_start_h = lf_roll_h;
			lf_pitch_start_h = lf_pitch_h;
			lf_yaw_start_h = lf_yaw_h;

			rf_roll_start_h = rf_roll_h;
			rf_pitch_start_h = rf_pitch_h;
			rf_yaw_start_h = rf_yaw_h;
		}
		firstRunPos = false;

		// compute delta position
		delta_p_h = pi_h - p_start_h;
		for (int i=0; i<3 ; i++){
			delta_p_r(i) = m_ratioLimbs(0)*delta_p_h(i);
		}
		for (int i=3; i<9 ; i++){
			delta_p_r(i) = m_ratioLimbs(1)*delta_p_h(i);
		}
		for (int i=9; i<15 ; i++){
			delta_p_r(i) = m_ratioLimbs(2)*delta_p_h(i);
		}

		delta_base_h = basei_h - base_start_h;
		delta_base_r = m_ratioLimbs(3) * delta_base_h;

		delta_roll = roll_h - roll_start_h;
		delta_pitch = pitch_h - pitch_start_h;
		delta_yaw = yaw_h - yaw_start_h;

		n_delta_roll = n_roll_h - n_roll_start_h;
		n_delta_pitch = n_pitch_h - n_pitch_start_h;
		n_delta_yaw = n_yaw_h - n_yaw_start_h;

		lf_delta_roll = lf_roll_h - lf_roll_start_h;
		lf_delta_pitch = lf_pitch_h - lf_pitch_start_h;
		lf_delta_yaw = lf_yaw_h - lf_yaw_start_h;

		rf_delta_roll = rf_roll_h - rf_roll_start_h;
		rf_delta_pitch = rf_pitch_h - rf_pitch_start_h;
		rf_delta_yaw = rf_yaw_h - rf_yaw_start_h;

		// pi_r = p_start_r + delta_p_r 
		//		= p_start_r + m * delta_p_h
		//		= p_start_r + m * (pi_h - p_start_h)
		if (start_pos.compare("T")==0){
		    pi_r = p_start_r_T + delta_p_r;
			bodySegPos = pi_r;
			basei_r = base_start_r_T + delta_base_r;
		}
		else{ // delta_p_r
			bodySegPos = delta_p_r;
			basei_r = delta_base_r;
		}
	}
	else {
		streamingPos = false;  
	}
}

//----------------------------
//-- xSens-Robot CoM Retargeting
//----------------------------
void retargetingThread::getXsensCoM()
{
	Bottle *input = com_port.read(false);
	if(input!=NULL){   
		streamingCoM = true;
		for (int i=0; i<3; i++){
			com(i) = input->get(i).asDouble();
		}
	}
	else {
		streamingCoM = false;	
	}
	//compute CoM offset from left_foot on the vector connecting the two feet
	if (streamingPos) {
		p_com(0) = com(0);
		p_com(1) = com(1);
		p_Lfoot(0) = l_foot(0);
		p_Lfoot(1) = l_foot(1);
		p_Rfoot(0) = r_foot(0);
		p_Rfoot(1) = r_foot(1);

		p_RLfeet = p_Rfoot - p_Lfoot;
		double norm;
		norm = abs(p_RLfeet(0)*p_RLfeet(0) + p_RLfeet(1)*p_RLfeet(1));

		o_com = ((p_com - p_Lfoot).dot(p_Rfoot - p_Lfoot))/norm;
		com(3) = o_com;
	}
	//compute DeltaCoM outside feet line 
	if (streamingJoint) {
		Eigen::VectorXd qdummy_r(actuatedDOFs);
		// copy retargeted joint values but keep same initial q for the legs
		qdummy_r = qstart_r;
		for (int i=0; i<20; i++){
			qdummy_r(i) = jointPos(i)+qstart_r(i);
		}
		icub_model.setAng(eigenToYarp(qdummy_r));
		dummyCom_ = icub_model.getCOM(link_index);

		deltaCom = dummyCom_(0) - dummyCom_start(0);
		com(4) = deltaCom;
	}

}


//------ RUN ------
void retargetingThread::run()
{
	getXsensCoM();
	getRobotJoints();
	getRobotPos();

	if (this->m_checkJointLimits)
		avoidJointLimits();

	double    t = yarp::os::Time::now();
	publishData();
}


void retargetingThread::publishData()
{
	if (streamingJoint){
		publishJoints();
	}
	if (streamingPos){
		publishPos();
	}
	if (streamingCoM){
		publishCoM();
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
	yInfo() << "Closing com port";
	com_port.interrupt();
	com_port.close();
	yInfo() << "com port closed";
}
