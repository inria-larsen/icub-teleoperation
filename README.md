# icub-teleoperation
[WORK IN PROGRESS]

## What is this

A collection of modules for the teleoperation of the iCub robot.
It implements the motion-retargeting from the xSens capture-motion system to the robot.


## Software requirements

On Ubuntu 16.04:
* yarp
* codyco-superbuild
* icub-main
* icub-contrib


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

## How to run the modules

### Concept

### Set-up procedure

Firstly, the user must guarantee that both the host computer (the one running the yarp server), and the client computer (the one running xsens modules) are connected to the same network.

The yarp.conf file in both the computers has to be properly set-up:
- 'yarp conf', on both the computers to retrieve the yarp.conf files location.
- On Windows: 'ipconfig', to retrieve the ip address assigned to the computer.
- On Ubuntu: 'ifconfig'
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

### Launch procedure

The procedure to run the modules is still quite elaborate.


On Ubuntu:
- terminal 1. `yarp server`
- terminal 2. Bring the robot in a suitable home position (e.g. `$ yarpmotorgui --from homePoseRetarget.ini` and then pressing the 'Home All' button)
- terminal 3. Launch `torqueBalancing` or the controller you want to use
- terminal 4.5. `cd icub-teleoperation/build/bin`
- terminal 4. Launch `comMapping`
- terminal 5. Launch `jointMapping`
- terminal 6. Open a write port to to connect to the rpc module of the controller `yarp write /write`
- terminal 7. `yarp connect /write /torqueBalancing/rpc`
- terminal 7. Connect the output port of comMapping to the input com-reference port of the controller `yarp connect /xsensToRobot/com:o /torqueBalancing/comDes:i`
- terminal 7. Connect the output port of jointMapping to the input joint-reference port of the controller `yarp connect /xsensToRobot/q /torqueBalancing/qDes:i`

On Windows:
- Open MVN xsens
- Connect the suit or open a recorded file.
- Run xsens.exe
- On another terminal. `yarp connect /xsens/JointAngles /xsensToRobot/com:i`
- `yarp connect /xsens/JointAngles /xsensToRobot/q`

On Ubuntu again:
- terminal 6. `start`

On Windows again: (skip this step if using the suit)
- Play button on recording



