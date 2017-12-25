/*
 * Copyright (C) 2015 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef _SIMPLE_LEGGED_ODOMETRY_
#define _SIMPLE_LEGGED_ODOMETRY_

#include <iCub/iDynTree/DynTree.h>

/**
 * This class implements a simple legged odometry scheme for a generic robot.
 *
 * Under the assumption that at least a link of the robot at the time is
 * not moving (no slippage), it computes the estimate of the transform
 * between a inertial/world frame and the robot floating base.
 *
 * The algorithm implemented is the following :
 *
 * 1a) At start (or at reset) the user of the class specifies:
 *       * a frame (world) that should be assumed as the inertial/world frame
 *       * a frame (fixed) that is rigidly attached to a link that is not moving
 *         with respect to the specified inertial frame.
 *     At the start, the world_H_fixed (${}^{world} H_{fixed}$) transfomr between this two specified
 *     frames will be saved.
 * 1b) At this point, the getWorldToFrameTransform(int frame_id) will return the world_H_frame
 *      ( ${}^{world} H_{frame}$ ) transform simply by computing the forward kinematics from the fixed frame
 *     to the frame specified by frame_id : world_H_frame = world_H_fixed * fixed_H_frame(qj)
 *                                          ${}^{world} H_{frame} = {}^{world} H_{fixed} {}^{fixed} H_{frame}(qj)$
 * 2) If the fixed frame changes, we can simply change the frame used as "fixed", and consistently update the
 *      world_H_fixed transform to be equal to world_H_new_fixed =  world_H_old_fixed * old_fixed_H_new_fixed(qj) :
 *      ${}^{world} H_{fixed} = {}^{world} H_{old_fixed} {}^{old_fixed} H_{new_fixed}(qj)$
 * 2b) After the update, the getWorldToFrameTransform(int frame_id) can be obtained as at the point 1b .
 *
 * \note we should update the state used by the odometry by calling the appropritate getDynTree().setAng() method.
 */
class simpleLeggedOdometry
{
    private:
        iCub::iDynTree::DynTree * odometry_model;
        int current_fixed_link_id;
        KDL::Frame world_H_fixed;

    public:
	simpleLeggedOdometry():
	    odometry_model(0),
	    current_fixed_link_id(-1),
	    world_H_fixed()
	{

	}

	~simpleLeggedOdometry()
	{
	    if( odometry_model )
	    {
		delete odometry_model;
		odometry_model=0;
	    }
	}

	bool init(KDL::CoDyCo::UndirectedTree & undirected_tree,
		                        const std::string& initial_world_frame_position,
		                        const std::string& initial_fixed_link)
	{
	    int initial_world_frame_position_index = undirected_tree.getLink(initial_world_frame_position)->getLinkIndex();
	    int initial_fixed_link_index = undirected_tree.getLink(initial_fixed_link)->getLinkIndex();;
	    if( initial_fixed_link_index < 0 ||
		initial_world_frame_position_index < 0 )
	    {
		return false;
	    }

	    return init(undirected_tree,initial_world_frame_position_index,initial_fixed_link_index);
	}

	bool init(KDL::CoDyCo::UndirectedTree & undirected_tree,
		                        const int initial_world_frame_position_index,
		                        const int initial_fixed_link_index)
	{
	    if( odometry_model )
	    {
		delete odometry_model;
		odometry_model=0;
	    }

	    odometry_model = new iCub::iDynTree::DynTree(undirected_tree.getTree(),undirected_tree.getSerialization());
	    bool ok = reset(initial_world_frame_position_index,initial_fixed_link_index);
	    return ok;
	}

	bool reset(const std::string& initial_world_frame_position, const std::string& initial_fixed_link)
	{
	    int initial_world_frame_position_index = odometry_model->getLinkIndex(initial_world_frame_position);
	    int initial_fixed_link_index = odometry_model->getLinkIndex(initial_fixed_link);
	    if( initial_fixed_link_index < 0 ||
		initial_world_frame_position_index < 0 )
	    {
		return false;
	    }

	    return reset(initial_world_frame_position_index,initial_fixed_link_index);
	}

	bool reset(const int initial_world_frame_position_index, const int initial_fixed_link_index)
	{
	    current_fixed_link_id = initial_fixed_link_index;
	    world_H_fixed = odometry_model->getPositionKDL(initial_world_frame_position_index,initial_fixed_link_index);
	    return true;
	}


		/**
		 * Change the link that the odometry assumes to be fixed with the
		 * inertial/world frame
		 */
	bool changeFixedLink(const std::string& new_fixed_link_name)
	{
	    int new_fixed_link_id = odometry_model->getLinkIndex(new_fixed_link_name);

	    if( new_fixed_link_id < 0 )
	    {
		return false;
	    }

	    return changeFixedLink(new_fixed_link_id);
	}

		/**
		 * Change the link that the odometry assumes to be fixed with the
		 * inertial/world frame
		 */
	bool changeFixedLink(const int& new_fixed_link_id)
	{
	    int old_fixed_link_id = this->current_fixed_link_id;
	    KDL::Frame world_H_old_fixed = this->world_H_fixed;
	    KDL::Frame old_fixed_H_new_fixed = odometry_model->getPositionKDL(old_fixed_link_id,new_fixed_link_id);
	    this->world_H_fixed = world_H_old_fixed*old_fixed_H_new_fixed;
	    this->current_fixed_link_id = new_fixed_link_id;
	    return true;
	}

		/**
		 * Get the link currently considered fixed with rispect to the inertial frame.
		 * @return the name of the link currently considered fixed.
		 */
	std::string getCurrentFixedLink()
	{
	    std::string ret_string;
	    odometry_model->getLinkName(this->current_fixed_link_id,ret_string);
	    return ret_string;
	}
		/**
		 * Get the world_H_frame transform.
		 *
		 */
	KDL::Frame getWorldFrameTransform(const int frame_index)
	{
	    KDL::Frame fixed_H_frame = odometry_model->getPositionKDL(this->current_fixed_link_id,frame_index);
	    return this->world_H_fixed*fixed_H_frame;
	}

		/**
		 * Set the joint positions, velocities and accelerations
		 */
	bool setJointsState(const KDL::JntArray& qj,
		                                  const KDL::JntArray& dqj,
		                                  const KDL::JntArray& ddqj)
	{
	    if( qj.rows() != static_cast<unsigned int>(odometry_model->getNrOfDOFs())  ||
		dqj.rows() != static_cast<unsigned int>(odometry_model->getNrOfDOFs())  ||
		ddqj.rows() != static_cast<unsigned int>(odometry_model->getNrOfDOFs()) )
	    {
		return false;
	    }

	    bool ok = true ;

	    ok = ok && odometry_model->setAngKDL(qj);
	    ok = ok && odometry_model->setDAngKDL(dqj);
	    ok = ok && odometry_model->setD2AngKDL(ddqj);

	    //Update also the floating base position, given this new joint positions
	    KDL::Frame world_H_base = this->getWorldFrameTransform(odometry_model->getFloatingBaseLink());
	    ok = ok && odometry_model->setWorldBasePoseKDL(world_H_base);

	    return ok;
	}
		/**
		 * Get iDynTree underlyng object
		 *
		 */
	const iCub::iDynTree::DynTree & getDynTree(){
	    return *odometry_model;
	}
};

#endif
