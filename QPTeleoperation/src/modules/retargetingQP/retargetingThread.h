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
             
	/*Joint related*/
	Eigen::VectorXd jointPos;
        int actuatedDOFs;
	bool m_checkJointLimits;
        Eigen::VectorXd m_minJointLimits; /* actuatedDOFs */
        Eigen::VectorXd m_maxJointLimits; /* actuatedDOFs */
	/*end Joint related*/
        Eigen::VectorXd bodySegPos;
	
	//Port for reading and writing the joint positions
        yarp::os::BufferedPort<yarp::os::Bottle> joint_port; 
        //Port for reading and writing the body segment positions
        yarp::os::BufferedPort<yarp::os::Bottle> pos_port;

	void mapping_run();
	//----------------------------
	//-- xSens-Robot Joint Retargeting
	//----------------------------
	void getRobotJoints();
	void getRobotPos();
	void avoidJointLimits();
	void publishJoints();
	void publishPos();
	  

public:

    	retargetingThread(std::string _name,
			   std::string _robotName,
			   int _actuatedDOFs,
			   bool checkJointLimits,
                           Eigen::VectorXd minJointLimits,
                           Eigen::VectorXd maxJointLimits,
			   int _period,
			   double _offset);
	
	bool threadInit();
	void run();
	void closePort();

};


#endif
