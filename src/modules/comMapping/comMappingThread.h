/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef COM_MAPPING_THREAD
#define COM_MAPPING_THREAD

// System includes
#include <cstring>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <map>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>

// Yarp includes
#include <yarp/os/Time.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Mutex.h>

// iDynTree includes
#include "iCub/iDynTree/yarp_kdl.h"
#include <iCub/iDynTree/TorqueEstimationTree.h>

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>

#include "robotStatus.h"

// Local includes
#include "simpleLeggedOdometry.h"

namespace wbi {
    class ID;
    class IDList;
}

using namespace yarp::sig;
using namespace yarp::os;
using namespace wbi;
using namespace yarpWbi;
using namespace std;
using namespace Eigen;

const double PI = 3.141592653589793;



//===============================================
//        ComMapping Main THREAD (runs every 10ms)
//===============================================

class comMappingThread: public yarp::os::RateThread
{
	/** prefix for all the ports opened by this module */
	std::string moduleName;
	/** prefix for all the ports of the robot at which we are connecting */
	std::string robotName;
	/** wholeBodySensors interface to get sensors readings */
	wbi::iWholeBodySensors * sensors;

	/** helper variable for printing every printPeriod milliseconds */
	int                 printCountdown;
	/** period after which some diagnostic messages are print */
	double              printPeriod;

	yarp::os::Stamp timestamp;


	void closePort(yarp::os::Contactable *_port)
	{
	    if (_port)
	    {
		_port->interrupt();
		_port->close();

		delete _port;
		_port = 0;
	    }
	}


	void estimation_run()
	{
	    //Compute odometry
	    publishOdometry();

	    //if normal mode, publish the
	    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

	    if( printCountdown == 0 ) {
	       
		double avgTime, stdDev, avgTimeUsed, stdDevUsed;

		getEstPeriod(avgTime, stdDev);
		getEstUsed(avgTimeUsed, stdDevUsed);

	    }

	}

	yarp::os::Mutex run_mutex;
	bool run_mutex_acquired;
	RobotJointStatus joint_status;
	VectorXd jointPos;
	yarp::sig::Vector q;

	iCub::iDynTree::TorqueEstimationTree * icub_model_calibration;
	iCub::iDynTree::TorqueEstimationTree * icub_model_world_base_position;
	iCub::iDynTree::TorqueEstimationTree * icub_model_calibration_on_l_sole;
	iCub::iDynTree::TorqueEstimationTree * icub_model_calibration_on_r_sole;
	std::string calibration_support_link;

	int left_foot_link_idyntree_id;
	int right_foot_link_idyntree_id;
	int root_link_idyntree_id;

	// iCubGui related variables
	int icubgui_support_frame_idyntree_id;
	KDL::Frame initial_world_H_supportFrame;

	bool assume_fixed_base_calibration;
	std::string fixed_link_calibration;
	bool assume_fixed_base_calibration_from_odometry;

	yarp::os::Property yarp_options;

	//---------------------------------------------------------
        //   simpleLeggedOdometry private attributes and methods
        //---------------------------------------------------------
	simpleLeggedOdometry odometry_helper;
	int odometry_floating_base_frame_index;
	yarp::sig::Matrix world_H_floatingbase;
	bool odometry_enabled;
	yarp::os::BufferedPort<yarp::sig::Vector> * port_com;
        BufferedPort<Bottle> port; /**<Port that reads the joint status*/
	std::string current_fixed_link_name;
    
	bool initOdometry()
	{
	    yarp::os::Bottle & odometry_group = yarp_options.findGroup("SIMPLE_LEGGED_ODOMETRY");

	    if( odometry_group.isNull()  )
	    {
		yInfo() << " SIMPLE_LEGGED_ODOMETRY group not found, odometry disabled";
		this->odometry_enabled = false;
		return true;
	    }

	    if( !odometry_group.check("initial_world_frame") ||
		!odometry_group.check("initial_fixed_link") ||
		!odometry_group.check("floating_base_frame") ||
		!odometry_group.find("initial_world_frame").isString() ||
		!odometry_group.find("initial_fixed_link").isString() ||
		!odometry_group.find("floating_base_frame").isString() )
	    {
		yError() << " SIMPLE_LEGGED_ODOMETRY group found but malformed, exiting";
		this->odometry_enabled = false;
		return false;
	    }

	    std::string initial_world_frame = odometry_group.find("initial_world_frame").asString();
	    std::string initial_fixed_link = odometry_group.find("initial_fixed_link").asString();
	    std::string floating_base_frame = odometry_group.find("floating_base_frame").asString();

	    // Allocate model
	    KDL::CoDyCo::UndirectedTree undirected_tree = this->icub_model_calibration->getKDLUndirectedTree();
	    bool ok = this->odometry_helper.init(undirected_tree,
		                                 initial_world_frame,
		                                 initial_fixed_link);
	    this->current_fixed_link_name = initial_fixed_link;
	 //   externalWrenchTorqueEstimator->current_fixed_link_name = initial_fixed_link;

	    // Get floating base frame index
	    this->odometry_floating_base_frame_index = odometry_helper.getDynTree().getFrameIndex(floating_base_frame);

	    ok = ok && (this->odometry_floating_base_frame_index >= 0 &&
		        this->odometry_floating_base_frame_index < odometry_helper.getDynTree().getNrOfFrames());

	    if( !ok )
	    {
		yError() << "Odometry initialization failed, please check your parameters";
		return false;
	    }

	    yInfo() << " SIMPLE_LEGGED_ODOMETRY initialized with initial world frame coincident with "
		   << initial_world_frame << " and initial fixed link " << initial_fixed_link;

	    this->odometry_enabled = true;
	    world_H_floatingbase.resize(4,4);

	    // Open ports
	    port_com = new BufferedPort<Vector>;
	    port_com->open(string("/"+moduleName+"/com:o").c_str());

	    return true;
	}


	void publishOdometry()
	{
	    if( this->odometry_enabled )
	    {
		// Read joint position, velocity and accelerations into the odometry helper model
		// This could be avoided by using the same geometric model
		// for odometry, force/torque estimation and sensor force/torque calibration
		odometry_helper.setJointsState(joint_status.getJointPosKDL(),
		                               joint_status.getJointVelKDL(),
		                               joint_status.getJointAccKDL());

		// Get floating base position in the world
		KDL::Frame world_H_floatingbase_kdl = odometry_helper.getWorldFrameTransform(this->odometry_floating_base_frame_index);

		// Publish the floating base position on the port
		KDLtoYarp_position(world_H_floatingbase_kdl,this->world_H_floatingbase);

		// Stream com in world frame
		KDL::Vector com = odometry_helper.getDynTree().getCOMKDL();

		yarp::sig::Vector & com_to_send = port_com->prepare();
		com_to_send.resize(3);

		KDLtoYarp(com,com_to_send);

		port_com->write();
	 
		// save the current link considered as fixed by the odometry
		current_fixed_link_name = odometry_helper.getCurrentFixedLink();
	    }
	}
	    

	void closeOdometry()
	{
	    closePort(port_com);
	}

	public:

	    comMappingThread(string _name,
		                                         string _robotName,
		                                         int _period,
		                                         yarpWholeBodySensors *_wbs,
		                                         yarp::os::Property & _yarp_options,
		                                         bool _assume_fixed_base_calibration,
		                                         std::string _fixed_link_calibration,
		                                         bool _assume_fixed_base_calibration_from_odometry
		                                        )
	    :  RateThread(_period),
	       moduleName(_name),
	       robotName(_robotName),
	       sensors(_wbs),
	       yarp_options(_yarp_options),
	       printCountdown(0),
	       printPeriod(2000),
	       assume_fixed_base_calibration(_assume_fixed_base_calibration),
	       fixed_link_calibration(_fixed_link_calibration),
	       assume_fixed_base_calibration_from_odometry(_assume_fixed_base_calibration_from_odometry),
	       run_mutex_acquired(false),
	       odometry_enabled(false)
	{
		// TODO FIXME move all this logic in threadInit

	       yInfo() << "Launching comMappingThread with name : " << _name << " and robotName " << _robotName << " and period " << _period;

	       if( !_yarp_options.check("urdf") )
	       {
		    yError() << "comMapping error: urdf not found in configuration files";
		    return;
	       }

	    std::string urdf_file = _yarp_options.find("urdf").asString().c_str();
	    yarp::os::ResourceFinder rf;
	    if( _yarp_options.check("verbose") )
	    {
		rf.setVerbose();
	    }

	    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());

	    std::vector<std::string> dof_serialization;

	    // \todo TODO FIXME move IDList -> std::vector<std::string> conversion to wbiIdUtils
	    IDList torque_estimation_list = sensors->getSensorList(wbi::SENSOR_ENCODER);
	    for(int dof=0; dof < (int)torque_estimation_list.size(); dof++)
	    {
		ID wbi_id;
		torque_estimation_list.indexToID(dof,wbi_id);
		dof_serialization.push_back(wbi_id.toString());
	    }

	    std::vector<std::string> ft_serialization;
	    IDList ft_sensor_list =  _wbs->getSensorList(wbi::SENSOR_FORCE_TORQUE);
	    for(int ft=0; ft < (int)ft_sensor_list.size(); ft++)
	    {
		ID wbi_id;
		ft_sensor_list.indexToID(ft,wbi_id);
		ft_serialization.push_back(wbi_id.toString());
	    }

	       if( assume_fixed_base_calibration ) {
		   icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,fixed_link_calibration);
		   icub_model_world_base_position = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,fixed_link_calibration);
	       }
	       else if( assume_fixed_base_calibration_from_odometry )
	       {
		   icub_model_calibration_on_l_sole = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,"l_sole");
		   icub_model_calibration_on_r_sole = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization,"r_sole");
		   icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
	       }
	       else
	       {
		   icub_model_calibration = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
		   icub_model_world_base_position = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
	       }
	}


	bool threadInit()
	{
	    joint_status.setNrOfDOFs(icub_model_calibration->getNrOfDOFs());

	    //Find end effector ids
	    int max_id = 100;

	    root_link_idyntree_id = icub_model_calibration->getLinkIndex("root_link");
	    //yAssert(root_link_idyntree_id >= 0 && root_link_idyntree_id < max_id );
	    left_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("l_foot");
	    //yAssert(left_foot_link_idyntree_id >= 0  && left_foot_link_idyntree_id < max_id);
	    right_foot_link_idyntree_id = icub_model_calibration->getLinkIndex("r_foot");
	    //yAssert(right_foot_link_idyntree_id >= 0 && right_foot_link_idyntree_id < max_id);
	    joint_status.zero();

	    if( assume_fixed_base_calibration )
	    {
		icubgui_support_frame_idyntree_id = icub_model_calibration->getLinkIndex(fixed_link_calibration);

		if( icubgui_support_frame_idyntree_id < 0 )
		{
		    yError() << "Warning: unknown fixed link " << fixed_link_calibration;
		    return false;
		}
	    }
	    else
	    {
		icubgui_support_frame_idyntree_id = left_foot_link_idyntree_id;
	    }

	    icub_model_calibration->setAng(joint_status.getJointPosYARP());
	    //{}^world H_{leftFoot}
	    initial_world_H_supportFrame
		    = icub_model_calibration->getPositionKDL(root_link_idyntree_id,icubgui_support_frame_idyntree_id);
	    
	   //opening reading port
	   port.open(string("/"+moduleName+"/com:i").c_str());

	   //---------------------------------------------------------
           //   Odometry initialization
           //---------------------------------------------------------
	    initOdometry();

	    yInfo() << "comMappingThread::threadInit finished successfully.";

	    return true;
	}


	void getRobotJoints()
	{
	/*  //Don't wait to get a sensor measure
	    bool wait = false;
	    //Don't get timestamps
	    double * stamps = NULL;

            // Get joint encoders position, velocities and accelerations
	    sensors->readSensors(wbi::SENSOR_ENCODER_POS, joint_status.getJointPosKDL().data.data(), stamps, wait);
	    sensors->readSensors(wbi::SENSOR_ENCODER_SPEED, joint_status.getJointVelKDL().data.data(), stamps, wait);
	    sensors->readSensors(wbi::SENSOR_ENCODER_ACCELERATION, joint_status.getJointAccKDL().data.data(), stamps, wait);
	*/

	    //read joint angles value from xsens port
            Bottle *input = port.read();  //to move to getData()

            double neck_pitch = (input->get(17).asDouble())*-1;
	    double neck_roll = input->get(15).asDouble();
	    double neck_yaw = input->get(16).asDouble();
	    double torso_yaw = (input->get(4).asDouble()+input->get(7).asDouble()+input->get(10).asDouble())*-1;
	    double torso_roll = (input->get(6).asDouble()+input->get(9).asDouble())*-1;
	    double torso_pitch = (input->get(8).asDouble()+input->get(5).asDouble()+input->get(2).asDouble());
	    double l_shoulder_pitch = (input->get(35).asDouble())*-1;
	    double l_shoulder_roll = input->get(33).asDouble();
	    double l_shoulder_yaw = input->get(34).asDouble();
	    double l_elbow = input->get(38).asDouble();
	    double l_wrist_prosup = (input->get(40).asDouble())*-1;
	    double l_wrist_pitch = (input->get(41).asDouble())*-1;
	    double l_wrist_yaw = (input->get(39).asDouble())*-1;
	    double r_shoulder_pitch = (input->get(23).asDouble())*-1;
	    double r_shoulder_roll = input->get(21).asDouble();
	    double r_shoulder_yaw = input->get(22).asDouble();
	    double r_elbow = input->get(26).asDouble();
	    double r_wrist_prosup = (input->get(28).asDouble())*-1;
	    double r_wrist_pitch = (input->get(29).asDouble())*-1;
	    double r_wrist_yaw = (input->get(27).asDouble())*-1;
	    double l_hip_pitch = input->get(56).asDouble();
	    double l_hip_roll = input->get(54).asDouble();
	    double l_hip_yaw = (input->get(55).asDouble())*-1;
	    double l_knee = (input->get(59).asDouble())*-1;
	    double l_ankle_pitch = (input->get(62).asDouble())*-1;
	    double l_ankle_roll = input->get(60).asDouble();
	    double r_hip_pitch = input->get(44).asDouble();
	    double r_hip_roll = input->get(42).asDouble();
	    double r_hip_yaw = (input->get(43).asDouble())*-1;
	    double r_knee = (input->get(47).asDouble())*-1;
	    double r_ankle_pitch = (input->get(50).asDouble())*-1;
	    double r_ankle_roll = input->get(48).asDouble();
		
	    jointPos.resize(32);
	    jointPos << torso_pitch*PI/180, torso_roll*PI/180, torso_yaw*PI/180, neck_pitch*PI/180, neck_roll*PI/180, neck_yaw*PI/180, l_shoulder_pitch*PI/180, l_shoulder_roll*PI/180, l_shoulder_yaw*PI/180, l_elbow*PI/180, l_wrist_prosup*PI/180, l_wrist_pitch*PI/180, l_wrist_yaw*PI/180, r_shoulder_pitch*PI/180, r_shoulder_roll*PI/180, r_shoulder_yaw*PI/180, r_elbow*PI/180, r_wrist_prosup*PI/180, r_wrist_pitch*PI/180, r_wrist_yaw*PI/180, l_hip_pitch*PI/180, l_hip_roll*PI/180, l_hip_yaw*PI/180, l_knee*PI/180, l_ankle_pitch*PI/180, l_ankle_roll*PI/180, r_hip_pitch*PI/180, r_hip_roll*PI/180, r_hip_yaw*PI/180, r_knee*PI/180, r_ankle_pitch*PI/180, r_ankle_roll*PI/180;

	    q.resize(jointPos.size());
	    for(int i =0; i < (q.size()); i++)
	    {
		q(i) = jointPos(i);
            }	    

	    joint_status.setJointPosYARP(q);
	    // Update yarp vectors
	    joint_status.updateYarpBuffers();
	}
	   

        //------ RUN ------
	void run()
	{
	    if( this->run_mutex_acquired )
	    {
		yError() << "comMapping: run_mutex already acquired at the beginning of run method.";
		yError() << "    this could cause some problems, please report an issue at https://github.com/robotology/codyco-modules/issues/new";
	    }

	    run_mutex.lock();
	    this->run_mutex_acquired = true;
	    getRobotJoints();

	    estimation_run();
	   
	    this->run_mutex_acquired = false;
	    run_mutex.unlock();
	}
	   

	void threadRelease()
	{
	    run_mutex.lock();

	    yInfo() << "Deleting icub models used for calibration";
	    delete icub_model_calibration;

	    if( this->assume_fixed_base_calibration_from_odometry )
	    {
		delete icub_model_calibration_on_l_sole;
		delete icub_model_calibration_on_r_sole;
	    }

	    if( !this->assume_fixed_base_calibration_from_odometry )
	    {
		delete icub_model_world_base_position;
	    }

	    yInfo() << "Closing odometry class";
	    closeOdometry();

	    run_mutex.unlock();
	}

};


#endif
