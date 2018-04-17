/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "jointRetargetingModule.h"
#include "jointRetargetingThread.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarpWbi;
using namespace wbi;


jointRetargetingModule::jointRetargetingModule()
	:  m_robot(0)
{
    jointThread      = 0;
    offset 	    = 0.1;
    period          = 10;
}


bool jointRetargetingModule::configure(yarp::os::ResourceFinder &rf) 
{
    if( rf.check("robot") ) {
	robotName = rf.find("robot").asString();
    } else {
	std::cerr << "jointRetargetingModule::configure failed: robot parameter not found. Closing module." << std::endl;
	return false;
    }

    if( rf.check("name") ) {
	moduleName = rf.find("name").asString();
	setName(moduleName.c_str());
    } else {
	std::cerr << "jointRetargetingModule::configure failed: name parameter not found. Closing module." << std::endl;
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

    //Loading joints information
    yarp::os::Property wbiProperties;
    if (!rf.check("wbi_config_file", "Checking wbi configuration file")) {
        yError("No WBI configuration file found.");
        return false;
    }

    if (!rf.check("wbi_joint_list", "Checking wbi joint list name")) {
        yError("No joint list found. Please specify a joint list in \"wbi_joint_list\"");
        return false;
   }

    if (!wbiProperties.fromConfigFile(rf.findFile("wbi_config_file"))) {
        yError("Not possible to load WBI properties from file.");
        return false;
    }
    wbiProperties.fromString(rf.toString(), false);

    yarp::os::ConstString jointList = rf.find("wbi_joint_list").asString();
    //retrieve all main joints
    wbi::IDList iCubMainJoints;
    if (!yarpWbi::loadIdListFromConfig(jointList, wbiProperties, iCubMainJoints)) {
        yError("Cannot find joint list");
        return false;
    }
    actuatedDOFs = iCubMainJoints.size();
    Value trueValue;
    trueValue.fromString("true");
    bool checkJointLimits = rf.check("check_limits", trueValue, "Looking for joint limits check option").asBool();

    //create reference to wbi
    m_robot = new yarpWbi::yarpWholeBodyInterface(moduleName.c_str(), wbiProperties);
    if (!m_robot) {
       yError("Could not create wbi object.");
        return false;
    }

    //add joints
    m_robot->addJoints(iCubMainJoints);
    if (!m_robot->init()) {
        yError("Could not initialize wbi.");
        return false;
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
    

    //--------------------------JOINT RETARGETING THREAD--------------------------
    jointThread = new jointRetargetingThread(moduleName,
	                                    robotName,
					    actuatedDOFs,
					    *m_robot,
					    checkJointLimits,
					    wbiProperties,
	                                    period,
					    offset);
    if(!jointThread->start())
    {
	yError() << getName()
	                  << ": Error while initializing whole body estimator thread."
	                  << "Closing module";
	return false;
    }

    yInfo("jointRetargetingThread started. (Running at %d ms)",period);


    return true;
} 

    
bool jointRetargetingModule::interruptModule()                       
{
    if(jointThread)
	jointThread->suspend();
    rpcPort.interrupt();
    return true;
}


bool jointRetargetingModule::close()                                 
{
    // Get for the last time time stats
    jointThread->getEstPeriod(avgTime, stdDev);
    jointThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
	
    //stop threads
    if(jointThread)
    {
	yInfo() << getName() << ": closing jointRetargetingThread";
	jointThread->stop();
	delete jointThread;
	jointThread = 0;
    }

    //closing ports
    std::cout << getName() << ": closing RPC port interface" << std::endl;
    rpcPort.close();


    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*period)
	printf("Next time you could set a lower period to improve the jointRetargeting performance.\n");
    else if(avgTime>1.3*period)
	printf("The period you set was impossible to attain. Next time you could set a higher period.\n");


    return true;
}


bool jointRetargetingModule::updateModule()
{
    if (jointThread==0)
    {
	yError("jointRetargetingThread pointers are zero\n");
	return false;
    }

    jointThread->getEstPeriod(avgTime, stdDev);
    jointThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    if(avgTime > 1.3 * period)
    {
	yWarning("[WARNING] jointRetargeting loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
	yInfo("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }


    return true;
}

 
bool jointRetargetingModule::quit()
{
    return this->close();
}
