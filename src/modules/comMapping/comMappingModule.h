/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __COM_MAPPING_MODULE_H__
#define __COM_MAPPING_MODULE_H__

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
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <yarp/os/LogStream.h>

#include "wholeBodyDynamics_IDLServer.h"

#include "comMappingThread.h"

#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include "yarpWholeBodyInterface/yarpWholeBodyInterface.h"


using namespace yarp::dev;
using namespace yarpWbi;
using namespace yarp::os;
using namespace wbi;

class comMappingModule: public yarp::os::RFModule, public wholeBodyDynamics_IDLServer
{
    /* module parameters */
    std::string  moduleName;
    std::string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

    yarp::os::Port                 rpcPort;        // a port to handle rpc messages
    comMappingThread*     comThread;     // locomotion control thread
    yarpWbi::yarpWholeBodySensors* sensors;

public:
	comMappingModule(){
	    comThread      = 0;
	    sensors  = 0;
	    period          = 10;
	}


	bool attach(yarp::os::Port &source)          // Attach the module to a RPC port
	{
	    return this->yarp().attachAsServer(source);
	}


	bool configure(yarp::os::ResourceFinder &rf) // configure all the module parameters and return true if successful
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


	    bool fixed_base = false;
	    bool fixed_base_calibration = false;
	    std::string fixed_link;
	    std::string fixed_link_calibration;
	    if( rf.check("assume_fixed") )
	    {
		fixed_link = rf.find("assume_fixed").asString().c_str();
		if( fixed_link != "root_link" &&
		    fixed_link != "l_sole" &&
		    fixed_link != "r_sole" &&
		    fixed_link != "r_foot_dh_frame" &&
		    fixed_link != "l_foot_dh_frame" )
		{
		    yError() << "assume_fixed option found, but disabled because " << fixed_link << " is not a recognized fixed_link ";
		    return false;
		} else {
		    yInfo() << "assume_fixed option found, using " << fixed_link << " as fixed link as a kinematic root instead of the imu.";
		    fixed_base = true;
		    fixed_base_calibration = true;
		    fixed_link_calibration = fixed_link;
		    // \todo TODO workaround for heidelberg
		    if( fixed_link == "l_sole" )
		    {
		        fixed_link = fixed_link_calibration = "l_foot_dh_frame";
		    }

		    if( fixed_link == "r_sole" )
		    {
		        fixed_link = fixed_link_calibration = "r_foot_dh_frame";
		    }
		}
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

	    //Add to the options some wbd specific stuff
	    if( fixed_base )
	    {
		yarpWbiOptions.put("fixed_base",fixed_link);
	    }

	    if( rf.check("assume_fixed_from_odometry") )
	    {
		yarpWbiOptions.put("assume_fixed_from_odometry","dummy");
	    }
	  
	    if( rf.check("calibration_support_link") )
	    {
		yarpWbiOptions.put("calibration_support_link",rf.find("calibration_support_link").asString());
	    }
	    else
	    {
		yarpWbiOptions.put("calibration_support_link","root_link");
	    }

	    sensors = new yarpWholeBodySensors(robotName.c_str(), yarpWbiOptions);

	    sensors->addSensors(wbi::SENSOR_ENCODER,RobotDynamicModelJoints);

	     //List of 6-axis Force-Torque sensors in the robot
	    IDList RobotFTSensors;
	    std::string RobotFTSensorsListName = "ROBOT_MAIN_FTS";
	    if( !loadIdListFromConfig(RobotFTSensorsListName,yarpWbiOptions,RobotFTSensors) )
	    {
		yError("comMapping: impossible to load wbiId list with name %s\n",RobotFTSensorsListName.c_str());
	    }
	    sensors->addSensors(wbi::SENSOR_FORCE_TORQUE,RobotFTSensors);

	    //List of IMUs sensors in the robot
	    IDList RobotIMUSensors;
	    std::string RobotIMUSensorsListName = "ROBOT_MAIN_IMUS";
	    if( !loadIdListFromConfig(RobotIMUSensorsListName,yarpWbiOptions,RobotIMUSensors) )
	    {
		yError("comMapping: impossible to load wbiId list with name %s\n",RobotFTSensorsListName.c_str());
	    }
	    sensors->addSensors(wbi::SENSOR_IMU,RobotIMUSensors);

	    if(!sensors->init())
	    {
		yError() << getName() << ": Error while initializing whole body estimator interface.Closing module";
		return false;
	    }

	    

	    //--------------------------WHOLE BODY DYNAMICS THREAD--------------------------
	    comThread = new comMappingThread(moduleName,
		                                    robotName,
		                                    period,
		                                    sensors,
		                                    yarpWbiOptions,
		                                    fixed_base_calibration,
		                                    fixed_link_calibration,
		                                    rf.check("assume_fixed_from_odometry"));
	    if(!comThread->start())
	    {
		yError() << getName()
		                  << ": Error while initializing whole body estimator thread."
		                  << "Closing module";
		return false;
	    }

	    yInfo() << "comMappingThread started";


	    return true;
	} 

	    
	bool interruptModule()                       // interrupt, e.g., the ports
	{
	    if(comThread)
		comThread->suspend();
	    rpcPort.interrupt();
	    return true;
	}


	bool close()                                 // close and shut down the module
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
	    if(sensors)
	    {
		yInfo() << getName() << ": closing wholeBodySensors interface";
		bool res=sensors->close();
		if(!res)
		    yError("Error while closing robot sensors interface\n");
		delete sensors;
		sensors = 0;
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


	double getPeriod(){ 
            return period;  
        }


	bool updateModule()
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

	    /**
	     * Quit the module.
	     * @return true/false on success/failure
	     */
	virtual bool quit()
	{
	    return this->close();
	}
};



#endif
//empty line to make gcc happy

