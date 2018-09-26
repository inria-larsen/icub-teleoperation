/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "retargetingModule.h"
#include "retargetingThread.h"

using namespace yarp::dev;
using namespace yarp::os;


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
	   start_pos = rf.find("start_pos").asString();
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

    //Loading joint value format
    if( rf.check("joint_value") && rf.find("joint_value").isString() )
    {
       joint_value = rf.find("joint_value").asString();
    }

    //Loading robot specific pose - relative body segment positions
    Bottle *p_start_r_ = rf.find("p_start_r").asList(); 
    size = p_start_r_->size();
    p_start_r.resize(size);
    for (int i=0; i<size; i++){
        p_start_r(i) = p_start_r_->get(i).asDouble();
    }

    //Loading robot specific pose - joint angles
    Bottle *j_start_r_ = rf.find("j_start_r").asList(); 
    size = j_start_r_->size();
    j_start_r.resize(size);
    for (int i=0; i<size; i++){
        j_start_r(i) = j_start_r_->get(i).asDouble();
    }

    //Loading robot specific pose - base z
    if( rf.check("base_start_r") && rf.find("base_start_r").isDouble() )
    {
       base_start_r = rf.find("base_start_r").asDouble();
    }

    // //dummy robot values
    // Bottle *q_start_r_ = rf.find("j_start_dummy").asList(); 
    // size = q_start_r_->size();
    // q_start_r.resize(size);
    // for (int i=0; i<size; i++){
    //     q_start_r(i) = q_start_r_->get(i).asDouble();
    // }
    // if( rf.check("urdf") && rf.find("urdf").isString() )
    // {
    //    urdf_file_path = rf.find("urdf").asString();
    // }



    //--------------------------RPC PORT--------------------------------------------
    attach(rpcPort);
    std::string rpcPortName= "/";
    rpcPortName+= getName();
    rpcPortName += "/rpc:i";
    if (!rpcPort.open(rpcPortName.c_str())) {
	   std::cerr << getName() << ": Unable to open port " << rpcPortName << std::endl;
	   return false;
    }
    

    //--------------------------RETARGETING THREAD--------------------------
    rThread = new retargetingThread(moduleName,
	                                    robotName,
                					    actuatedDOFs,
                					    checkJointLimits,
                                        m_minJointLimits,
                                        m_maxJointLimits,
	                                    period,
                					    offset,
                					    m_ratioLimbs,
                					    j_start_r,
				                        p_start_r,
                                        base_start_r,
                                        ref_frame,
				                        start_pos,
                                        joint_value);
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




