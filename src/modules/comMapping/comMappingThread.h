/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef COM_MAPPING_THREAD
#define COM_MAPPING_THREAD

// System includes
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

// Yarp includes
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
#include <iCub/iDynTree/TorqueEstimationTree.h>

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>

// Local includes
#include "simpleLeggedOdometry.h"
#include "robotStatus.h"

using namespace Eigen;

class comMappingThread: public yarp::os::RateThread
{
	/** prefix for all the ports opened by this module */
	std::string moduleName;
	/** prefix for all the ports of the robot at which we are connecting */
	std::string robotName;
	/** wholeBodySensors interface to get sensors readings */
	wbi::iWholeBodySensors * sensors;
	/** helper variable for printing every printPeriod milliseconds */
	int                 printCountdown;
	/** period after which some diagnostic messages are print */
	double              printPeriod;
	yarp::os::Stamp timestamp;

	void closePort(yarp::os::Contactable *_port);
	void mapping_run();

	yarp::os::Mutex run_mutex;
	bool run_mutex_acquired;
	RobotJointStatus joint_status;
	VectorXd jointPos;
	yarp::sig::Vector q;

	iCub::iDynTree::TorqueEstimationTree * icub_model;

	int left_foot_link_idyntree_id;
	int right_foot_link_idyntree_id;
	int root_link_idyntree_id;

	// iCubGui related variables
	//int icubgui_support_frame_idyntree_id;
	//KDL::Frame initial_world_H_supportFrame;

	yarp::os::Property yarp_options;

	// simpleLeggedOdometry private attributes and methods
	simpleLeggedOdometry odometry_helper;
	bool odometry_enabled;
	yarp::os::BufferedPort<yarp::sig::Vector> * port_com;
	yarp::os::BufferedPort<yarp::os::Bottle> port; /**<Port that reads the joint status*/
	std::string current_fixed_link_name;

	bool initOdometry();
	void publishCom();
	void closeOdometry();

public:

    comMappingThread(std::string _name,
		     std::string _robotName,
		     int _period,
		     yarpWbi::yarpWholeBodySensors *_wbi,
		     yarp::os::Property & _yarp_options);

	bool threadInit();
	void getRobotJoints();
	void run();
	void threadRelease();

};


#endif
