/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef LIP_THREAD
#define LIP_THREAD

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


class LIPThread: public yarp::os::RateThread
{
	/** prefix for all the ports opened by this module */
	std::string moduleName;
	/** prefix for all the ports of the robot at which we are connecting */
	std::string robotName;
	/** helper variable for printing every printPeriod milliseconds */
	int                 printCountdown;
	/** period after which some diagnostic messages are print */
	double              printPeriod;
	yarp::os::Stamp timestamp;
	bool streamingCoM=false;

	bool firstRun=true;

    //Port for reading and writing CoM information
    yarp::os::BufferedPort<yarp::os::Bottle> com_port;

    // Data from the controller
    Eigen::VectorXd qpdata;

    // Rate time controller
    double T;

    // Z CoM
    double hc;
    
    // Reg factor zmp norm
    Eigen::Vector2d lambdaD;

    // Matrices for cost function
    Eigen::MatrixXd costFunctionH;
    Eigen::VectorXd costFunctionF;

    // Matrices for the stacked constraints
    Eigen::Vector2d bConstraintMax;
    Eigen::Vector2d bConstraintMin;

    // State
    Eigen::Vector2d comPos;
    Eigen::Vector2d comVel;
    Eigen::Vector2d comAcc;
    Eigen::Vector2d zmpPos;

    //Desired values
    Eigen::Vector2d comPos_des;
    Eigen::Vector2d comVel_des;
    Eigen::Vector2d comAcc_des;

    // Stabilized values
    Eigen::Vector2d comPos_stab;
    Eigen::Vector2d comVel_stab;

    // solution of the QP
    Eigen::VectorXd zmp;

    // zmp associated to the actual CoM
    Eigen::VectorXd zmp_actcom;


	void publishData();
	void LIPretarget();
    Eigen::VectorXd solveQP();
    void getQPdata();
    void publishStableCoM();

	  

public:

	LIPThread(std::string _name,
			   std::string _robotName,
			   int _period,
			   Eigen::Vector2d m_lambdaD);
	
	bool threadInit();
	void run();
	void closePort();

};


#endif
