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

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>


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
    std::string ref_frame;
    Eigen::VectorXd m_ratioLimbs; 
    Eigen::VectorXd p_start_r_T;  
    Eigen::VectorXd p_start_h;
    std::string start_pos; 
    bool stream_feet;
    Eigen::VectorXd l_foot;
    Eigen::VectorXd r_foot;
	/*CoM related*/
	Eigen::VectorXd com;
	double o_com; // com offset from left foot on the vector connecting the two feet
	Eigen::Vector2d p_com;
	Eigen::Vector2d p_Lfoot;
	Eigen::Vector2d p_Rfoot;

	
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
               std::string _ref_frame,
               std::string _start_pos,
               bool _stream_feet,
               std::string _joint_value);
	
	bool threadInit();
	void run();
	void closePort();

};


#endif
