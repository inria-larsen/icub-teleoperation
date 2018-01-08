/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

// System includes
#include <cstring>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

// Yarp includes
#include <yarp/os/Time.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/SVD.h>

// iDynTree includes
#include "iCub/iDynTree/yarp_kdl.h"

// Local includes
#include "comMappingThread.h"

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

comMappingThread::comMappingThread(string _name,
                                         string _robotName,
                                         int _period,
	  				 int _nDOFs,
			       		 wbi::wholeBodyInterface& robot,
			       		 bool checkJointLimits,
                                         yarpWholeBodySensors *_wbs,
                                         yarp::os::Property & _yarp_options,
					 double _offset)
    :  RateThread(_period),
       moduleName(_name),
       robotName(_robotName),
       nDOFs(_nDOFs),
       m_robot(robot),
       sensors(_wbs),
       yarp_options(_yarp_options),
       m_checkJointLimits(checkJointLimits),	
       offset(_offset),
       printCountdown(0),
       printPeriod(2000),
       run_mutex_acquired(false),
       odometry_enabled(false),
       m_minJointLimits(_nDOFs),
       m_maxJointLimits(_nDOFs)
{

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
    icub_model = new iCub::iDynTree::TorqueEstimationTree(urdf_file_path,dof_serialization,ft_serialization);
  
}


bool comMappingThread::threadInit()
{
    joint_status.setNrOfDOFs(icub_model->getNrOfDOFs());

    //Find end effector ids
    int max_id = 100;

    root_link_idyntree_id = icub_model->getLinkIndex("root_link");
    //yAssert(root_link_idyntree_id >= 0 && root_link_idyntree_id < max_id );
    left_foot_link_idyntree_id = icub_model->getLinkIndex("l_foot");
    //yAssert(left_foot_link_idyntree_id >= 0  && left_foot_link_idyntree_id < max_id);
    right_foot_link_idyntree_id = icub_model->getLinkIndex("r_foot");
    //yAssert(right_foot_link_idyntree_id >= 0 && right_foot_link_idyntree_id < max_id);
    joint_status.zero();

    //icubgui_support_frame_idyntree_id = left_foot_link_idyntree_id;

    //limits
    bool result = false;
    result = m_robot.getJointLimits(m_minJointLimits.data(), m_maxJointLimits.data());
    if (!result) {
        yError("Failed to compute joint limits.");
        return false;
    }

    if (this->m_checkJointLimits) {
        std::cout << "[INFO]Joint limits are:\nmin" <<m_minJointLimits.transpose() << "\nmax " << m_maxJointLimits.transpose() << "\n";
    } else {
        yInfo("Joint limits disabled");
    }

    icub_model->setAng(joint_status.getJointPosYARP());
    //{}^world H_{leftFoot}
    ///initial_world_H_supportFrame = icub_model->getPositionKDL(root_link_idyntree_id,icubgui_support_frame_idyntree_id);
    
   //opening reading port
   port.open(string("/xsensToRobot/com:i").c_str());

   //---------------------------------------------------------
   //   Odometry initialization
   //---------------------------------------------------------
    initOdometry();

    yInfo() << "comMappingThread::threadInit finished successfully.";

    return true;
}


bool comMappingThread::initOdometry()
{
    yarp::os::Bottle & odometry_group = yarp_options.findGroup("SIMPLE_LEGGED_ODOMETRY");

    if( odometry_group.isNull()  )
    {
	yInfo() << " SIMPLE_LEGGED_ODOMETRY group not found, odometry disabled";
	this->odometry_enabled = false;
	return true;
    }

    if( !odometry_group.check("initial_world_frame") ||
	!odometry_group.check("fixed_link") ||
	!odometry_group.check("floating_base_frame") ||
	!odometry_group.find("initial_world_frame").isString() ||
	!odometry_group.find("fixed_link").isString() ||
	!odometry_group.find("floating_base_frame").isString() )
    {
	yError() << " SIMPLE_LEGGED_ODOMETRY group found but malformed, exiting";
	this->odometry_enabled = false;
	return false;
    }

    std::string initial_world_frame = odometry_group.find("initial_world_frame").asString();
    std::string fixed_link = odometry_group.find("fixed_link").asString();
    std::string floating_base_frame = odometry_group.find("floating_base_frame").asString();

    // Allocate model
    KDL::CoDyCo::UndirectedTree undirected_tree = this->icub_model->getKDLUndirectedTree();
    bool ok = this->odometry_helper.init(undirected_tree,
	                                 initial_world_frame,
	                                 fixed_link);
    this->current_fixed_link_name = fixed_link; 

    if( !ok )
    {
	yError() << "Odometry initialization failed, please check your parameters";
	return false;
    }

    yInfo() << " SIMPLE_LEGGED_ODOMETRY initialized with initial world frame coincident with "
	   << initial_world_frame << " and  fixed link " << fixed_link;

    this->odometry_enabled = true;

    // Open ports
    port_com = new BufferedPort<Vector>;
    port_com->open(string("/xsensToRobot/com:o").c_str());

    return true;
}


void comMappingThread::publishCom()
{
    if( this->odometry_enabled )
    {
	// Read joint position, velocity and accelerations into the odometry helper model
	// This could be avoided by using the same geometric model
	// for odometry, force/torque estimation and sensor force/torque calibration
	odometry_helper.setJointsState(joint_status.getJointPosKDL(),
	                               joint_status.getJointVelKDL(),
	                               joint_status.getJointAccKDL());


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
    

void comMappingThread::closeOdometry()
{
    closePort(port_com);
}


//============================
//-- xSens-Robot Joint Mapping
//============================
void comMappingThread::getRobotJoints()
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
    Bottle *input = port.read();  

    //-------------------------------
    //-- xSens to iCub
    if (robotName.find("icub") != std::string::npos){
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
	
	    jointPos.resize(nDOFs);
    jointPos << torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll;
    }
    //----------------------------------------------------------
    else {
	yError() << "[ERROR] xSens-robot joint mapping not defined (implementation of getRobotJoints() required!)";
    	return;
    }

    // Tranform to radiants
    q.resize(jointPos.size());
    for(int i =0; i < (q.size()); i++)
    {
	q(i) = jointPos(i)*PI/180;
    }

    if (this->m_checkJointLimits){
	    // Avoid joint limits
	    for (int i = 0; i < q.size(); i++){ 
	    	if (q(i) <= (m_minJointLimits(i) + offset))		//check min
			q(i) = m_minJointLimits(i) + offset;
		if (q(i) >= (m_maxJointLimits(i) - offset))		//check max
			q(i) = m_maxJointLimits(i) - offset;
	    }    
    }
    joint_status.setJointPosYARP(q);
    // Update yarp vectors
    joint_status.updateYarpBuffers();
}
   
//-----------------
//------ RUN ------
//-----------------
void comMappingThread::run()
{
    if( this->run_mutex_acquired )
    {
	yError() << "comMapping: run_mutex already acquired at the beginning of run method.";
	yError() << "    this could cause some problems, please report an issue at https://github.com/robotology/codyco-modules/issues/new";
    }

    run_mutex.lock();
    this->run_mutex_acquired = true;
    getRobotJoints();

    mapping_run();
   
    this->run_mutex_acquired = false;
    run_mutex.unlock();
}
   

void comMappingThread::mapping_run()
{
    //Compute com
    publishCom();

    //if normal mode, publish the
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)

    if( printCountdown == 0 ) {
       
	double avgTime, stdDev, avgTimeUsed, stdDevUsed;

	getEstPeriod(avgTime, stdDev);
	getEstUsed(avgTimeUsed, stdDevUsed);

    }

}


void comMappingThread::threadRelease()
{
    run_mutex.lock();

    yInfo() << "Deleting icub model used";
    delete icub_model;

    yInfo() << "Closing odometry class";
    closeOdometry();

    run_mutex.unlock();
}


void comMappingThread::closePort(yarp::os::Contactable *_port)
{
    if (_port)
    {
	_port->interrupt();
	_port->close();

	delete _port;
	_port = 0;
    }
}


