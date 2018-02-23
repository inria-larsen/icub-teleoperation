# icub-teleoperation

Work in progress about the teleoperation of the iCub

## What is this

A collection of modules for the teleoperation of the iCub robot.
It implements the motion-retargeting from the Xsens motion-capture system to the robot.


## Software requirements

On Ubuntu 16.04:
* yarp
* icub-main
* icub-contrib
* gazebo
* codyco-superbuild

To see how to install all the software on Ubuntu 16, check [here](https://github.com/inria-larsen/icub-manual/wiki/How-to-install-the-software-on-your-machine-(Ubuntu-16))

On Windows:
* xsens-yarp streaming module
* mvn xsens


## Installation

```
git clone https://github.com/inria-larsen/icub-teleoperation
cd icub-teleoperation
mkdir build
cd build
cmake ..
make
```


## Concept

The data from the motion capture system MVN Xsens is used as a reference for the robot motion. However, direct mapping from human to humanoids is not possible, due to differences in kinematics (e.g.joint limits, body dimensions) and dynamics (e.g. mass distribution). 
Hence, as a first step you can use a retargeting module, to obtain joint angles references congruent with the robot and to generate corresponding CoM position references. The output of the module is sent as a reference to the controller [torqueBalancing](https://www.frontiersin.org/articles/10.3389/frobt.2015.00006/full#B25). This retargeting framework is human-adapted in the sense that it works under the assumption that the motion performed by the human operator is a stable motion for the robot.
An improvement of the retargeting can be obtained by a cascade of the QP controller, which allows to compute a a weighted average of the desired tasks, generating a motion that is a comprise between the robot capabilities and the human reference, and the torque based controller that allows the robot to maintain the balance under the change of rigid non-coplanar contacts. 

![alt text](https://github.com/inria-larsen/icub-teleoperation/blob/master/doc/retargeting-chart.png "Software concept")


## How to run the modules

### Set-up procedure

#### Communication on yarp

Firstly, the user must guarantee that both the host computer (the one running the yarp server), and the client computer (the one running xsens modules) are connected to the same network.

The yarp.conf file in both the computers has to be properly set-up:
- `yarp conf`, on both the computers to retrieve the yarp.conf files location.
- On Windows: `ipconfig`, to retrieve the ip address assigned to the computer.
- On Ubuntu: `ifconfig`
- Edit the yarp.conf files:
	
	on the host computer

	```
	<host-ip-address> 10000 yarp
	```

	on the client computer

	```
	<host-ip-address> 10000 yarp
	<client-ip-address> 10000 yarp
	```

#### MVN Xsens

- Check that the right `<client-ip-address>` is assigned to the variable `server_name` in ../xsens/yarp/src/main.cpp
- Open MVN Xsens and ensure steaming is enabled:

    ->Options->Miscellaneous->network streamer

    check the following:

     [x] `<client-ipaddress>` 9763 UDP 0 0 [ ]

     [x] joint angles



### Human-adapted kinetic Retargeting - Launch procedure

#### From a live motion or from a recorded Xsense sequence 

On Ubuntu:
- terminal 1. `yarp server` (with --write option if necessary)
- Set the environmental variable `YARP_ROBOT_NAME` in the .bashrc according to the robot one wants to use (e.g. icubGazeboSim for simulations, or iCubNancy01, etc. for experiments).
- (real robot only) terminal 2. Bring the robot in a suitable home position (e.g. `$ yarpmotorgui --from homePoseRetarget.ini` and then pressing the 'Home All' button)
- (simulation only) terminal 2. Launch gazebo. If you want to use the synchronization between the controller and the simulator to avoid real-time factor related problems, launch gazebo as follows: `gazebo -slibgazebo_yarp_clock.so`

You can also try:
`cd icub-teleoperation/utilities/world` and `gazebo -slibgazebo_yarp_clock.so icub_retarget.world`

- terminal 3. `cd icub-teleoperation/build/bin`
- terminal 3. Launch `retargeting`
- terminal 4. Launch `wholeBodyDynamics` as follows: `YARP_ROBOT_NAME=icubGazeboSim yarprobotinterface --config launch-wholebodydynamics.xml`
- terminal 5 (OPTIONAL) type on a terminal `yarp rpc /wholeBodyDynamics/rpc` and execute the command `resetOffset all 300`. It will reset offsets of fake FT measurements, that might be affected by the results of a previous simulation. Fake FT measurements are used e.g. for defining the threshold for switching from single to double support balancing
- terminal 6. Launch Matlab (`matlab_codyco` to avoid problems with libstdc++. This is further discussed in [WB-Toolbox](https://github.com/robotology/WB-Toolbox) and in this [issue](https://github.com/robotology/codyco-superbuild/issues/141#issuecomment-257892256)
- Open the simulink model in `icub-teleoperation/controllers/torqueBalancing` and run the module.


On Windows:
- Open MVN Xsens
- Connect the suit or open a recorded file.
- (if wearing the suit) IMPORTANT. Start in the same initial pose assumed by the robot (`homePoseRetarget.ini`, i.e. n-pose). 
- Run xsens.exe
- On another terminal. `yarp connect /xsens/JointAngles /retargeting/q:i`
- (if using a recorded sequence) Play button on the recording on MVN Xsens


#### From a recorded sequence on Ubuntu only 

Follow the same steps as in the above procedure on Ubuntu and then:
- terminal 8. Launch `yarpdataplayer`
- ->File->Open Directory->retargetdata and select the folder of the sequence you want to play
- terminal 8. Connect the output streamed by yarpdataplayer to the input of the `retargeting` module `yarp connect /xsens/JointAngles /retargeting/q:i`
- Play button on yarpdataplayer


### Configuration files

Be sure that the conf file of the controller respects the following (e.g. `gains.m` or `torqueBalancing.ini`):
- `period` of the communicating modules has to be the same (it works for 10ms)
- the controller TorqueBalancing has been tested and works with the following gains
```
comKp       (50 50 50)

comKd       (15 15 15)

comKi       (0 0 0)

kImp        (90 90 90    20 20 20    20 20 20 20 20 20 20   20 20 20 20 20 20 20   30 30 30 60 10 10      30 30 30 60 10 10 )

```
`CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT` is an array of two binary values, each for the contact status of the left and right foot respectively. 0 or 1 if the foot is not or is in contact with the ground   

`CONFIG.SMOOTH_DES_COM`  If equal to one, the desired streamed values of the center of mass are smoothed for the first 50 frames 

`CONFIG.SMOOTH_DES_Q`    If equal to one, the desired streamed values  of the postural tasks are smoothed for the first 50 frames 

`references.smoothingTimeMinJerkComDesQDes` smoothing factor    

