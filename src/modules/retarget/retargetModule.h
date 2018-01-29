/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __RETARGET_H__
#define __RETARGET_H__
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <string.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include "wholeBodyDynamics_IDLServer.h"

#include <yarp/os/LogStream.h>

#include "yarpWholeBodyInterface/yarpWholeBodyInterface.h"

#include "retargetThread.h"


class retargetModule: public yarp::os::RFModule, public wholeBodyDynamics_IDLServer
{
    /* module parameters */
    std::string  moduleName;
    std::string  robotName;
    int     period;
    double  offset;
    int     actuatedDOFs;
    bool    checkJointLimits;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

    yarp::os::Port                 rpcPort;        // a port to handle rpc messages
    retargetThread*     rThread;     //  control thread
    wbi::wholeBodyInterface* m_robot;

public:
	retargetModule();

	bool attach(yarp::os::Port &source);          // Attach the module to a RPC port
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

