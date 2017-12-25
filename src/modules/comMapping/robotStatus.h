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
#include <iCub/iDynTree/yarp_kdl.h>

#include <wbi/iWholeBodySensors.h>

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
	bool zero()
	{
	    qj.zero();
	    dqj.zero();
	    ddqj.zero();
	    torquesj.zero();

	    SetToZero(qj_kdl);
	    SetToZero(dqj_kdl);
	    SetToZero(ddqj_kdl);
	    SetToZero(torquesj_kdl);

	    return true;
	}

	RobotJointStatus(int nrOfDOFs=0){
		    setNrOfDOFs(nrOfDOFs);

		    this->zero();
	}

	bool setNrOfDOFs(int nrOfDOFs)
	{

	    qj.resize(nrOfDOFs);
	    dqj.resize(nrOfDOFs);
	    ddqj.resize(nrOfDOFs);
	    torquesj.resize(nrOfDOFs);

	    qj_kdl.resize(nrOfDOFs);
	    dqj_kdl.resize(nrOfDOFs);
	    ddqj_kdl.resize(nrOfDOFs);
	    torquesj_kdl.resize(nrOfDOFs);

	    return zero();
	}

	bool setJointPosYARP(const yarp::sig::Vector & _qj)
	{
	    qj = _qj;
	    return YarptoKDL(qj,qj_kdl);
	}
	bool setJointVelYARP(const yarp::sig::Vector & _dqj)
	{
	    dqj = _dqj;
	    return YarptoKDL(dqj,dqj_kdl);
	}
	bool setJointAccYARP(const yarp::sig::Vector & _ddqj)
	{
	    ddqj = _ddqj;
	    return YarptoKDL(ddqj,ddqj_kdl);
	}
	bool setJointTorquesYARP(const yarp::sig::Vector & _torquesj)
	{
	    torquesj = _torquesj;
	    return YarptoKDL(torquesj,torquesj_kdl);
	}

	bool setJointPosKDL(const KDL::JntArray & _qj)
	{
	    qj_kdl = _qj;
	    return KDLtoYarp(qj_kdl,qj);
	}
	bool setJointVelKDL(const KDL::JntArray & _dqj)
	{
	    dqj_kdl = _dqj;
	    return KDLtoYarp(dqj_kdl,dqj);
	}
	bool setJointAccKDL(const KDL::JntArray & _ddqj)
	{
	    ddqj_kdl = _ddqj;
	    return KDLtoYarp(ddqj_kdl,ddqj);
	}
	bool setJointTorquesKDL(const KDL::JntArray & _torquesj)
	{
	    torquesj_kdl = _torquesj;
	    return KDLtoYarp(torquesj_kdl,torquesj);
	}

	yarp::sig::Vector & getJointPosYARP()
	{
	    return qj;
	}
	yarp::sig::Vector & getJointVelYARP()
	{
	    return dqj;
	}
	yarp::sig::Vector & getJointAccYARP()
	{
	    return ddqj;
	}
	yarp::sig::Vector & getJointTorquesYARP()
	{
	    return torquesj;
	}

	KDL::JntArray & getJointPosKDL()
	{
	    return qj_kdl;
	}
	KDL::JntArray & getJointVelKDL()
	{
	    return dqj_kdl;
	}
	KDL::JntArray & getJointAccKDL()
	{
	    return ddqj_kdl;
	}

	KDL::JntArray & getJointTorquesKDL()
	{
	    return torquesj_kdl;
	}

	    /**
	     * Copy in the yarp buffers the content of the KDL buffers.
	     */
	bool updateYarpBuffers()
	{
	    bool ok = true;
	    ok = ok && KDLtoYarp(qj_kdl,qj);
	    ok = ok && KDLtoYarp(dqj_kdl,dqj);
	    ok = ok && KDLtoYarp(ddqj_kdl,ddqj);
	    ok = ok && KDLtoYarp(torquesj_kdl,torquesj);
	    return ok;
	}

	    /**
	     * Copy in the KDL buffers the content of the YARP buffers.
	     */
	bool updateKDLBuffers()
	{
	    bool ok = true;
	    ok = ok && YarptoKDL(qj,qj_kdl);
	    ok = ok && YarptoKDL(dqj,dqj_kdl);
	    ok = ok && YarptoKDL(ddqj,ddqj_kdl);
	    ok = ok && YarptoKDL(torquesj,torquesj_kdl);
	    return ok;
	}
	};


#endif
