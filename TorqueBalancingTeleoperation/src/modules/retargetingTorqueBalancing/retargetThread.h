/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef RETARGET_THREAD
#define RETARGET_THREAD

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

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>


using namespace Eigen;


class retargetThread: public yarp::os::RateThread
{
        yarp::os::ResourceFinder rf;
	/** prefix for all the ports opened by this module */
	std::string moduleName;
	/** prefix for all the ports of the robot at which we are connecting */
	std::string robotName;
        std::string link_name;
        int link_index;
	double offset;
        int period;

	/** helper variable for printing every printPeriod milliseconds */
	int                 printCountdown;
	/** period after which some diagnostic messages are print */
	double              printPeriod;
	yarp::os::Stamp timestamp;
	wbi::wholeBodyInterface& m_robot;
        

	/*Joint related*/
	Eigen::VectorXd jointPosCoM;
	Eigen::VectorXd jointPos;
	std::string joint_list;
        int actuatedDOFs;
        yarp::sig::Vector q;
        yarp::sig::Vector qActuated;
        //Limits
	bool m_checkJointLimits;
        Eigen::VectorXd m_minJointLimits; /* actuatedDOFs */
        Eigen::VectorXd m_maxJointLimits; /* actuatedDOFs */
        std::string jointFormat;
	/*end Joint related*/
        yarp::sig::Vector dcom;
        yarp::sig::Vector ddcom;
        yarp::sig::Vector old_com;
        yarp::sig::Vector old_dcom;	

	iCub::iDynTree::DynTree icub_model;
	

        std::string controller;
	yarp::os::Mutex run_mutex;
	bool run_mutex_acquired;
	yarp::os::Property yarp_options;

        yarp::os::BufferedPort<yarp::os::Bottle> jointRead; /**<Port that reads the joint angles*/
        yarp::os::BufferedPort<yarp::os::Bottle> posRead; /**<Port that reads the segments position*/
        yarp::os::BufferedPort<yarp::sig::Vector> * port_com;
        yarp::os::BufferedPort<yarp::os::Bottle> port_joint;

        //----------------------------
        //* Initial status check-procedure 
        //* Control has to start in n-pose  
        //----------------------------
        bool safe_start;
        bool n_pose;
	yarp::os::Bottle* minJointSpan_yarp;
	yarp::os::Bottle* maxJointSpan_yarp;
        Eigen::VectorXd minJointSpan;        
        Eigen::VectorXd maxJointSpan;
        
        int counter;
        int frame_counter;
 

	bool initCoM();
        bool initJoint();
	void mapping_run();
	//----------------------------
	//-- xSens-Robot Joint Mapping
	//----------------------------
	void getRobotJoints();

	void avoidJointLimits();
        void publishCom();
	void publishJoints();
        void closePort(yarp::os::Contactable *_port);
	  

public:

    	retargetThread(std::string _name,
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
