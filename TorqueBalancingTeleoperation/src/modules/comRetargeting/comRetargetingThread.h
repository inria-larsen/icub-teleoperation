/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef COM_RETARGETING_THREAD
#define COM_RETARGETING_THREAD

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
#include <iCub/iDynTree/DynTree.h>

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>


using namespace Eigen;

class comRetargetingThread: public yarp::os::RateThread
{
	yarp::os::ResourceFinder rf;
	/** prefix for all the ports opened by this module */
	std::string moduleName;
	/** prefix for all the ports of the robot at which we are connecting */
	std::string robotName;
	std::string link_name;
        int link_index;
	double offset;
	/** wholeBody interface to get joint limits */
        wbi::wholeBodyInterface& m_robot;
	/** helper variable for printing every printPeriod milliseconds */
	int                 printCountdown;
	/** period after which some diagnostic messages are print */
	double              printPeriod;
	yarp::os::Stamp timestamp;

	void closePort(yarp::os::Contactable *_port);
	void retargeting_run();

	//yarp::os::Mutex run_mutex;
	//bool run_mutex_acquired;

	/*Joint related*/
	VectorXd jointPos;
	yarp::sig::Vector q;
	int nDOFs;
        //Limits
	bool m_checkJointLimits;
        Eigen::VectorXd m_minJointLimits; /* nDOFs */
        Eigen::VectorXd m_maxJointLimits; /* nDOFs */
	/*end Joint related*/

	iCub::iDynTree::DynTree icub_model;
        //module from which which this module gets the jointAgnles
        std::string input_module;

	yarp::os::Property yarp_options;
 

	// simpleLeggedOdometry private attributes and methods
	yarp::os::BufferedPort<yarp::sig::Vector> * port_com;
	yarp::os::BufferedPort<yarp::os::Bottle> port; /**<Port that reads the joint status*/

	bool initCom();
	void publishCom();
	void closeCom();

public:

    comRetargetingThread(std::string _name,
		     std::string _robotName,
		     int _period,
		     int _nDOFs,
		     wbi::wholeBodyInterface& robot,
		     bool checkJointLimits,
		     yarp::os::Property & _yarp_options,
		     double _offset);

	bool threadInit();
	void getRobotJoints();
	void run();
	void threadRelease();

};


#endif
