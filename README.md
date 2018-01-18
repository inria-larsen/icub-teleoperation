# icub-teleoperation

Work in progress about the teleoperation of the iCub

## What is this

A collection of modules for the teleoperation of the iCub robot.
It implements the motion-retargeting from the Xsens motion-capture system to the robot.


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

- Check that the right <client-ip-address> is assigned to the variable `server_name` in ../xsens/yarp/src/main.cpp
- Open MVN Xsens and ensure steaming is enabled:

    ->Options->Miscellaneous->network streamer

    check the following:

     [x] <client-ipaddress> 9763 UDP 0 0 [ ]

     [x] joint angles



### Launch procedure

#### From a live motion or from a recorded Xsense sequence 

On Ubuntu:
- terminal 1. `yarp server`
- terminal 2. Bring the robot in a suitable home position (e.g. `$ yarpmotorgui --from homePoseBalancing.ini` and then pressing the 'Home All' button)
- terminal 3. Launch `torqueBalancing` or the controller you want to use
- terminal 4.5. `cd icub-teleoperation/build/bin`
- terminal 4. Launch `comMapping`
- terminal 5. Launch `jointMapping`
- terminal 6. Open a write port to to connect to the rpc module of the controller `yarp write /write`
- terminal 7. `yarp connect /write /torqueBalancing/rpc`
- terminal 7. Connect the output port of comMapping to the input com-reference port of the controller `yarp connect /xsensToRobot/com:o /torqueBalancing/comDes:i`
- terminal 7. Connect the output port of jointMapping to the input joint-reference port of the controller `yarp connect /xsensToRobot/q /torqueBalancing/qDes:i`

On Windows:
- Open MVN Xsens
- Connect the suit or open a recorded file.
- Run xsens.exe
- On another terminal. `yarp connect /xsens/JointAngles /xsensToRobot/com:i`
- `yarp connect /xsens/JointAngles /xsensToRobot/q`

On Ubuntu again: (if using the suit, the operator wearing the suit has to assume the same home position of the robot)
- terminal 6. `start`

On Windows again: (skip this step if using the suit)
- Play button on the recording on MVN Xsens


#### From a recorded sequence on Ubuntu only 

- terminal 1. `yarp server`
- terminal 2. Bring the robot in a suitable home position (e.g. `yarpmotorgui --from homePoseRetarget.ini` and then pressing the 'Home All' button)
- terminal 3. Launch `torqueBalancing` or the controller you want to use
- terminal 4.5. `cd icub-teleoperation/build/bin`
- terminal 4. Launch `comMapping`
- terminal 5. Launch `jointMapping`
- terminal 6. Launch `yarpdataplayer`
- ->File->Open Directory->retargetdata and select the folder of the sequence you want to play
- terminal 7. Open a write port to to connect to the rpc module of the controller `yarp write /write`
- terminal 8. `yarp connect /write /torqueBalancing/rpc`
- terminal 8. Connect the output port of comMapping to the input com-reference port of the controller `yarp connect /xsensToRobot/com:o /torqueBalancing/comDes:i`
- terminal 8. Connect the output port of jointMapping to the input joint-reference port of the controller `yarp connect /xsensToRobot/q /torqueBalancing/qDes:i`
- terminal 8. Connect the output streamed by yarpdataplayer to the input of jointMapping `yarp connect /xsens/JointAngles /xsensToRobot/q`
- terminal 8. Connect the output streamed by yarpdataplayer to the input of comMapping `yarp connect /xsens/JointAngles /xsensToRobot/com:i`
- Play button on yarpdataplayer


### Configuration files

Be sure that the conf file of the controller respects the following (e.g. `torqueBalancing.ini`):
- `period` and `modulePeriod` are the same as the `period` in the conf files `comMapping.ini` and `jointMapping.ini`
- the controller TorqueBalancing has been tested and works with the following gains
```
comKp       (50 50 50)

comKd       (15 15 15)

comKi       (0 0 0)


kw          1

kImp        (100 100 100    10 10 10    20 20 20 20 20 20 20   20 20 20 20 20 20 20   30 30 30 60 10 10      30 30 30 60 10 10 )

tsat        (24 24 24    24 24 24    24 24 24 24 24 24 24   24 24 24 24 24 24 24   24 24 24 24 24 24      24 24 24 24 24 24 )
```


