/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef RETARGETING_THREAD
#define RETARGETING_THREAD

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
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Mutex.h>

// iDynTree includes
#include "iCub/iDynTree/yarp_kdl.h"
#include <iCub/iDynTree/DynTree.h>


using namespace Eigen;


class retargetingThread: public yarp::os::RateThread
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
	bool streamingJoint=false;
	bool streamingPos=false;
	bool streamingCoM=false;
	bool firstRunPos=true;
	bool firstRunJ=true;

	/*Joint related*/
	Eigen::VectorXd jointPos;
    int actuatedDOFs;
	bool m_checkJointLimits;
    Eigen::VectorXd m_minJointLimits; /* actuatedDOFs */
    Eigen::VectorXd m_maxJointLimits; /* actuatedDOFs */
    std::string joint_value;
    Eigen::VectorXd j_start_r_T;
    Eigen::VectorXd j_start_h;
    /*Body segment pose related*/
    Eigen::VectorXd bodySegPos;
    double basei_r;
    std::string ref_frame;
    Eigen::VectorXd m_ratioLimbs; 
    Eigen::VectorXd p_start_r_T;  
    Eigen::VectorXd p_start_h;
    double base_start_r_T;  
    double base_start_h;
    std::string start_pos; 
    bool stream_feet;
    bool stream_base;
    Eigen::VectorXd l_foot;
    Eigen::VectorXd r_foot;
    // base orientation
    double roll_start_h;
    double pitch_start_h;
    double yaw_start_h;
    double delta_roll;
    double delta_pitch;
    double delta_yaw;
    // neck orientation
    double n_roll_start_h;
    double n_pitch_start_h;
    double n_yaw_start_h;
    double n_delta_roll;
    double n_delta_pitch;
    double n_delta_yaw;
	/*CoM related*/
	Eigen::VectorXd com;
	double o_com; // com offset from left foot on the vector connecting the two feet
	Eigen::Vector2d p_com;
	Eigen::Vector2d p_Lfoot;
	Eigen::Vector2d p_Rfoot;
	Eigen::Vector2d p_RLfeet;

	/*dummy robot related*/
	iCub::iDynTree::DynTree icub_model;
	int link_index;
    Eigen::VectorXd qstart_r;
    std::string urdf_file_path;
    yarp::sig::Vector dummyCom_start;
    yarp::sig::Vector dummyCom_;
    double deltaCom; //duumy robot x com variation
	
	//Port for reading and writing the joint position
    yarp::os::BufferedPort<yarp::os::Bottle> joint_port; 
    //Port for reading and writing the body segment pose
    yarp::os::BufferedPort<yarp::os::Bottle> pos_port;
    //Port for reading and writing CoM information
    yarp::os::BufferedPort<yarp::os::Bottle> com_port;

	void publishData();
	//----------------------------
	//-- xSens-Robot Joint Retargeting
	//----------------------------
	void getRobotJoints();
	void getRobotPos();
	void getXsensCoM();
	void avoidJointLimits();
	void publishJoints();
	void publishPos();
	void publishCoM();

	Eigen::VectorXd toQuaternion(double pitch, double roll, double yaw)
	{
		Eigen::VectorXd q(4);
	        // Abbreviations for the various angular functions
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);

		q(0) = cy * cr * cp + sy * sr * sp;
		q(1) = cy * sr * cp - sy * cr * sp;
		q(2) = cy * cr * sp + sy * sr * cp;
		q(3) = sy * cr * cp - cy * sr * sp;
		return q;
	}

	double toRoll(const Eigen::VectorXd& q){
		double roll;
		// roll (x-axis rotation)
		double sinr = +2.0 * (q(0) * q(1) + q(2) * q(3));
		double cosr = +1.0 - 2.0 * (q(1) * q(1) + q(2) * q(2));
		roll = atan2(sinr, cosr);

		return roll;
	}

	double toPitch(const Eigen::VectorXd& q){
		double pitch;
		// pitch (y-axis rotation)
		double sinp = +2.0 * (q(0) * q(2) - q(3) * q(1));
		if (fabs(sinp) >= 1)
			pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
			pitch = asin(sinp);

		return pitch;
	}

	double toYaw(const Eigen::VectorXd& q){
		double yaw;
		// yaw (z-axis rotation)
		double siny = +2.0 * (q(0) * q(3) + q(1) * q(2));
		double cosy = +1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3));  
		yaw = atan2(siny, cosy);

		return yaw;
	}
	  
	yarp::sig::Vector eigenToYarp(const Eigen::VectorXd &eig){
		yarp::sig::Vector vec;
		vec.resize(eig.size());
		for (int i=0; i<eig.size(); i++){
			vec(i)=eig(i);
		}
		return vec;
	}

public:

	retargetingThread(std::string _name,
			   std::string _robotName,
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
               bool _base_feet,
               std::string _joint_value,
               Eigen::VectorXd qstart_r_,
               std::string urdf);
	
	bool threadInit();
	void run();
	void closePort();

};


#endif
