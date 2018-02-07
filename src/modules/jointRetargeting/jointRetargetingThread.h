/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef JOINT_RETARGETING_THREAD
#define JOINT_RETARGETING_THREAD

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


class jointRetargetingThread: public yarp::os::RateThread
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
	wbi::wholeBodyInterface& m_robot;
             
	/*Joint related*/
	Eigen::VectorXd jointPos;
	std::string joint_list;
        int actuatedDOFs;
        //Limits
	bool m_checkJointLimits;
        Eigen::VectorXd m_minJointLimits; /* actuatedDOFs */
        Eigen::VectorXd m_maxJointLimits; /* actuatedDOFs */
	/*end Joint related*/
	
	
        std::string controller;
	//yarp::os::Mutex run_mutex;
	//bool run_mutex_acquired;
	yarp::os::Property yarp_options;
        yarp::os::BufferedPort<yarp::os::Bottle> port; /**<Port for reading and writing the joint values*/


	bool initJoint();
	void mapping_run();
	//----------------------------
	//-- xSens-Robot Joint Retargeting
	//----------------------------
	void getRobotJoints();
	void avoidJointLimits();
	
	void publishJoints();
	  

public:

    	jointRetargetingThread(std::string _name,
			   std::string _robotName,
			   int _actuatedDOFs,
			   wbi::wholeBodyInterface& robot,
			   bool checkJointLimits,
		  	   yarp::os::Property & _yarp_options,
			   int _period,
			   double _offset);
	
	bool threadInit();
	void run();
	void threadRelease();

};


#endif
