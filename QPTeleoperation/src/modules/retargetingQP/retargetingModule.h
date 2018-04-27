/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __RETARGETING_MODULE_H__
#define __RETARGETING_MODULE_H__

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <string.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>

#include <yarp/os/LogStream.h>
#include "yarpWholeBodyInterface/yarpWholeBodyInterface.h"

#include "retargetingThread.h"


class retargetingModule: public yarp::os::RFModule
{
    /* module parameters */
    std::string  moduleName;
    std::string  robotName;
    int     period;
    /*Joint related*/
    double  offset;
    int     actuatedDOFs;
    bool    checkJointLimits;
    //yarp::os::Bottle* minJointLimits;
    Eigen::VectorXd m_minJointLimits; /* actuatedDOFs */
    Eigen::VectorXd m_maxJointLimits; /* actuatedDOFs */
    /*Body segment pose related*/
    std::string ref_frame;
    std::string start_pos;
    Eigen::VectorXd m_ratioLimbs;
    Eigen::VectorXd m_p_ref_T_r;
    Eigen::VectorXd m_p_ref_T_h;    

    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;
    
    yarp::os::Port                 rpcPort;        // a port to handle rpc messages
    retargetingThread*     rThread;     //  control thread

public:
	retargetingModule();

	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful    
	bool interruptModule();                       // interrupt, e.g., the ports
	bool close();                                 // close and shut down the module
	double getPeriod(){ return period;  }
	bool updateModule();
	
	    /**
	     * Quit the module.
	     * @return true/false on success/failure
	     */
	virtual bool quit();
	
};



#endif
//empty line to make gcc happy

