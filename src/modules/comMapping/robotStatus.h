/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef _WHOLE_BODY_DYNAMICS_ROBOT_STATUS_H_
#define _WHOLE_BODY_DYNAMICS_ROBOT_STATUS_H_

#include <yarp/sig/Vector.h>

#include <kdl/jntarray.hpp>

#include <vector>

class RobotJointStatus
{
    yarp::sig::Vector qj;
    yarp::sig::Vector dqj;
    yarp::sig::Vector ddqj;
    yarp::sig::Vector torquesj;

    KDL::JntArray qj_kdl;
    KDL::JntArray dqj_kdl;
    KDL::JntArray ddqj_kdl;
    KDL::JntArray torquesj_kdl;

public:
    bool zero();
    RobotJointStatus(int nrOfDOFs=0);
    bool setNrOfDOFs(int nrOfDOFs);

    bool setJointPosYARP(const yarp::sig::Vector & qj);
    bool setJointVelYARP(const yarp::sig::Vector & dqj);
    bool setJointAccYARP(const yarp::sig::Vector & ddqj);
    bool setJointTorquesYARP(const yarp::sig::Vector & torquesj);

    bool setJointPosKDL(const KDL::JntArray & qj);
    bool setJointVelKDL(const KDL::JntArray & dqj);
    bool setJointAccKDL(const KDL::JntArray & ddqj);
    bool setJointTorquesKDL(const KDL::JntArray & torquesj);

    yarp::sig::Vector & getJointPosYARP();
    yarp::sig::Vector & getJointVelYARP();
    yarp::sig::Vector & getJointAccYARP();
    yarp::sig::Vector & getJointTorquesYARP();

    KDL::JntArray & getJointPosKDL();
    KDL::JntArray & getJointVelKDL();
    KDL::JntArray & getJointAccKDL();
    KDL::JntArray & getJointTorquesKDL();

    /**
     * Copy in the yarp buffers the content of the KDL buffers.
     */
    bool updateYarpBuffers();

    /**
     * Copy in the KDL buffers the content of the YARP buffers.
     */
    bool updateKDLBuffers();
};


#endif
