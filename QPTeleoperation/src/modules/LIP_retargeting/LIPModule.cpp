/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "LIPModule.h"
#include "LIPThread.h"

using namespace yarp::os;

LIPModule::LIPModule()
{
    lThread      = 0;
    period          = 4;
}


bool LIPModule::configure(yarp::os::ResourceFinder &rf) 
{
    if( rf.check("robot") ) {
	   robotName = rf.find("robot").asString();
    } else {
	   std::cerr << "LIPModule::configure failed: robot parameter not found. Closing module." << std::endl;
	   return false;
    }

    if( rf.check("name") ) {
    	moduleName = rf.find("name").asString();
    	setName(moduleName.c_str());
    } else {
    	std::cerr << "LIPModule::configure failed: name parameter not found. Closing module." << std::endl;
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

    // Loading reg factor zmp norm
    if( rf.check("reg_factor_zmp") && rf.find("reg_factor_zmp").isList() )
    {
        Bottle *lambdaD = rf.find("reg_factor_zmp").asList(); 
        int size = lambdaD->size();
        m_lambdaD.resize(size);
        for (int i=0; i<size; i++)
            m_lambdaD(i) = lambdaD->get(i).asDouble();
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
    

    lThread = new LIPThread(moduleName, robotName, period, m_lambdaD);
    if(!lThread->start())
    {
	   yError() << getName()
	                  << ": Error while initializing LIP thread."
	                  << "Closing module";
	   return false;
    }

    yInfo("LIPThread started. (Running at %d ms)",period);


    return true;
} 

 
bool LIPModule::interruptModule()                       
{
    if(lThread)
    	lThread->suspend();
        rpcPort.interrupt();
    return true;
}


bool LIPModule::close()                                 
{
    // Get for the last time time stats
    lThread->getEstPeriod(avgTime, stdDev);
    lThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
    
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
    if(lThread)
    {
        yInfo() << getName() << ": closing LIPThread";
        lThread->closePort();
        lThread->stop();
        delete lThread;
        lThread = 0;
    }

    return true;
}


bool LIPModule::updateModule()
{
    if (lThread==0)
    {
    	yError("retargetingThread pointers are zero\n");
    	return false;
    }

    lThread->getEstPeriod(avgTime, stdDev);
    lThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()

    if(avgTime > 1.3 * period)
    {
    	yWarning("[WARNING] LIP loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
    	yInfo("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }
    return true;
}

 
bool LIPModule::quit()
{
    return this->close();
}




