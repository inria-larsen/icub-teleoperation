/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <string.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <yarp/os/LogStream.h>

#include "wholeBodyDynamics_IDLServer.h"

#include "comMappingModule.h"
#include "comMappingThread.h"

#include "yarpWholeBodyInterface/yarpWholeBodyInterface.h"


using namespace yarp::dev;
using namespace yarpWbi;
using namespace yarp::os;
using namespace wbi;

comMappingModule::comMappingModule()
	:  m_robot(0)
{
    comThread      = 0;
    period          = 10;
    offset 	    = 0.1;
}


bool comMappingModule::attach(yarp::os::Port &source) 
{
    return this->yarp().attachAsServer(source);
}


bool comMappingModule::configure(yarp::os::ResourceFinder &rf) 
{
    if( rf.check("robot") ) {
	robotName = rf.find("robot").asString();
    } else {
	std::cerr << "comMappingModule::configure failed: robot parameter not found. Closing module." << std::endl;
	return false;
    }

    if( rf.check("name") ) {
	moduleName = rf.find("name").asString();
	setName(moduleName.c_str());
    } else {
	std::cerr << "comMappingModule::configure failed: name parameter not found. Closing module." << std::endl;
	return false;
    }

    //Loading thread period
    if( rf.check("period") && rf.find("period").isInt() )
    {
	period = rf.find("period").asInt();
    }

    // If period is not specified, check also the legacy "rate" option
    if( !rf.check("period") && rf.check("rate") && rf.find("rate").isInt() )
    {
	period = rf.find("rate").asInt();
    }

    if( rf.check("offset") && rf.find("offset").isDouble() )
    {
	offset = rf.find("offset").asDouble();
    }


    //--------------------------RPC PORT--------------------------------------------
    attach(rpcPort);
    std::string rpcPortName= "/";
    rpcPortName+= getName();
    rpcPortName += "/rpc:i";
    if (!rpcPort.open(rpcPortName.c_str())) {
	std::cerr << getName() << ": Unable to open port " << rpcPortName << std::endl;
	return false;
    }

    //--------------------------WHOLE BODY STATES INTERFACE--------------------------
    yarp::os::Property yarpWbiOptions;
    //Get wbi options from the canonical file
    if( !rf.check("wbi_conf_file") )
    {
	fprintf(stderr,"[ERR] comMappingThread: impossible to open wholeBodyInterface: wbi_conf_file option missing");
	return false;
    }
    std::string wbiConfFile = rf.findFile("wbi_conf_file");
    yarpWbiOptions.fromConfigFile(wbiConfFile);

    //Overwrite the parameters in the wbi_conf_file with stuff from the wbd options file
    yarpWbiOptions.fromString(rf.toString(),false);

    //List of joints used in the dynamic model of the robot
    IDList RobotDynamicModelJoints;
    std::string RobotDynamicModelJointsListName = rf.check("torque_estimation_joint_list",
	                                                   yarp::os::Value("ROBOT_DYNAMIC_MODEL_JOINTS"),
	                                                   "Name of the list of joint used for torque estimation").asString().c_str();

    if( !loadIdListFromConfig(RobotDynamicModelJointsListName,rf,RobotDynamicModelJoints) )
    {
	if( !loadIdListFromConfig(RobotDynamicModelJointsListName,yarpWbiOptions,RobotDynamicModelJoints) )
	{
	    fprintf(stderr, "[ERR] comMappingModule: impossible to load wbiId joint list with name %s\n",RobotDynamicModelJointsListName.c_str());
	    return false;
	}
    }
    
    nDOFs = RobotDynamicModelJoints.size();

    Value trueValue;
    trueValue.fromString("true");
    bool checkJointLimits = rf.check("check_limits", trueValue, "Looking for joint limits check option").asBool();

    //create reference to wbi
    m_robot = new yarpWbi::yarpWholeBodyInterface(moduleName.c_str(), yarpWbiOptions);
    if (!m_robot) {
        yError("Could not create wbi object.");
        return false;
    }

    //add joints
    m_robot->addJoints(RobotDynamicModelJoints);
    if (!m_robot->init()) {
        yError("Could not initialize wbi.");
        return false;
    }

    

    

    //--------------------------COM MAPPING THREAD--------------------------
    comThread = new comMappingThread(moduleName,
	                                    robotName,
	                                    period,
					    nDOFs,
					    *m_robot,
					    checkJointLimits,
	                                    yarpWbiOptions,
					    offset);
    if(!comThread->start())
    {
	yError() << getName()
	                  << ": Error while initializing whole body estimator thread."
	                  << "Closing module";
	return false;
    }

    yInfo("comMappingThread started. (Running at %d ms)",period);


    return true;
} 

    
bool comMappingModule::interruptModule()      
{
    if(comThread)
	comThread->suspend();
    rpcPort.interrupt();
    return true;
}


bool comMappingModule::close()                                 
{
    // Get for the last time time stats
    comThread->getEstPeriod(avgTime, stdDev);
    comThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    //stop threads
    if(comThread)
    {
	yInfo() << getName() << ": closing comMappingThread";
	comThread->stop();
	delete comThread;
	comThread = 0;
    }
    if(m_robot)
    {
	yInfo() << getName() << ": closing robot interface";
	bool res=m_robot->close();
	if(!res)
	    yError("Error while closing robot model interface\n");
	delete m_robot;
	m_robot = 0;
    }

    //closing ports
    std::cout << getName() << ": closing RPC port interface" << std::endl;
    rpcPort.close();


    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*period)
	printf("Next time you could set a lower period to improve the comMapping performance.\n");
    else if(avgTime>1.3*period)
	printf("The period you set was impossible to attain. Next time you could set a higher period.\n");


    return true;
}


bool comMappingModule::updateModule()
{
    if (comThread==0)
    {
	yError("comMappingThread pointers are zero\n");
	return false;
    }

    comThread->getEstPeriod(avgTime, stdDev);
    comThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    if(avgTime > 1.3 * period)
    {
	yWarning("[WARNING] comMapping loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
	yInfo("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }


    return true;
}

    
bool comMappingModule::quit()
{
    return this->close();
}


