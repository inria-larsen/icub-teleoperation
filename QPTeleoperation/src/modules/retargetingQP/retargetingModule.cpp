/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "retargetingModule.h"
#include "retargetingThread.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarpWbi;
using namespace wbi;


retargetingModule::retargetingModule()
{
    rThread      = 0;
    offset 	    = 0.1;
    period          = 10;
}


bool retargetingModule::configure(yarp::os::ResourceFinder &rf) 
{
    if( rf.check("robot") ) {
	robotName = rf.find("robot").asString();
    } else {
	std::cerr << "retargetingModule::configure failed: robot parameter not found. Closing module." << std::endl;
	return false;
    }

    if( rf.check("name") ) {
	moduleName = rf.find("name").asString();
	setName(moduleName.c_str());
    } else {
	std::cerr << "retargetingModule::configure failed: name parameter not found. Closing module." << std::endl;
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

    //Loading safety offset from limits
    if( rf.check("offset") && rf.find("offset").isDouble() )
    {
	offset = rf.find("offset").asDouble();
    }

    //Loading ratio between icub and human wearing the xsens limbs
    Bottle *ratioLimbs = rf.find("ratio_limbs").asList(); 
    int size = ratioLimbs->size();
    m_ratioLimbs.resize(size);
    for (int i=0; i<size; i++){
        m_ratioLimbs(i) = ratioLimbs->get(i).asDouble();
    }

    //Loading ref frame for body segments
    if( rf.check("ref_frame") && rf.find("ref_frame").isString() )
    {
	ref_frame = rf.find("ref_frame").asString();
    }

    //Loading start pose
    if( rf.check("start_pos") && rf.find("start_pos").isString() )
    {
	ref_frame = rf.find("start_pos").asString();
    }
 
    //Loading joints information
    Bottle *jointList = rf.find("joint_list").asList();
    actuatedDOFs = jointList->size();
 std::cout << "[INFO]Joint list:\n" << jointList->toString() << std::endl;
    Value trueValue;
    trueValue.fromString("true");
    bool checkJointLimits = rf.check("check_limits", trueValue, "Looking for joint limits check option").asBool();
    //Loading limits
    Bottle *minJointLimits =rf.find("minJointLimits").asList();
    Bottle *maxJointLimits =rf.find("maxJointLimits").asList();
    m_minJointLimits.resize(actuatedDOFs);
    m_maxJointLimits.resize(actuatedDOFs);
    for (int i=0; i<actuatedDOFs; i++){
        m_minJointLimits(i) = minJointLimits->get(i).asDouble();
        m_maxJointLimits(i) = maxJointLimits->get(i).asDouble();
    }

    //Loading robot Tpose reference positions
    Bottle *p_ref_T_r = rf.find("p_ref_T_r").asList(); 
    size = p_ref_T_r->size();
    m_p_ref_T_r.resize(size);
    for (int i=0; i<size; i++){
        m_p_ref_T_r(i) = p_ref_T_r->get(i).asDouble();
    }
    //Loading human Tpose reference positions
    Bottle *p_ref_T_h = rf.find("p_ref_T_h").asList(); 
    size = p_ref_T_h->size();
    m_p_ref_T_h.resize(size);
    for (int i=0; i<size; i++){
        m_p_ref_T_h(i) = p_ref_T_h->get(i).asDouble();
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
    rThread = new retargetingThread(moduleName,
	                                    robotName,
					    actuatedDOFs,
					    checkJointLimits,
                                            m_minJointLimits,
                                            m_maxJointLimits,
	                                    period,
					    offset,
					    m_ratioLimbs,
					    m_p_ref_T_h,
				            m_p_ref_T_r,
                                            ref_frame,
					    start_pos);
    if(!rThread->start())
    {
	yError() << getName()
	                  << ": Error while initializing retargeting thread."
	                  << "Closing module";
	return false;
    }

    yInfo("retargetingThread started. (Running at %d ms)",period);


    return true;
} 

    
bool retargetingModule::interruptModule()                       
{
    if(rThread)
	rThread->suspend();
    rpcPort.interrupt();
    return true;
}


bool retargetingModule::close()                                 
{
    // Get for the last time time stats
    rThread->getEstPeriod(avgTime, stdDev);
    rThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
    
    //closing ports
    std::cout << getName() << ": closing RPC port interface" << std::endl;
    rpcPort.close();


    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*period)
	printf("Next time you could set a lower period to improve the retargeting performance.\n");
    else if(avgTime>1.3*period)
	printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

    //stop threads
    if(rThread)
    {
	yInfo() << getName() << ": closing retargetingThread";
        rThread->closePort();
	rThread->stop();
	delete rThread;
	rThread = 0;
    }

    return true;
}


bool retargetingModule::updateModule()
{
    if (rThread==0)
    {
	yError("retargetingThread pointers are zero\n");
	return false;
    }

    rThread->getEstPeriod(avgTime, stdDev);
    rThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    if(avgTime > 1.3 * period)
    {
	yWarning("[WARNING] retargeting loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
	yInfo("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }


    return true;
}

 
bool retargetingModule::quit()
{
    return this->close();
}




