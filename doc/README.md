## INSTRUCTIONS FOR LAUNCHING ON THE ROBOT


1. launch yarpmanager
2. launch yarpinterface with the WBD option (codyco)
3. launch iCubGui 
4. to be able to check the simmetry of the contact forces at the feet, visualize the robot in the air:
 
yarp write ... /iCubGui/base:i
>>0.0 0.0 0.0 0.0 0.0 1300.0

5. launch yarpmotorgui --from homePoseNancy.ini
this file is in /home/icub/software/src/codyco-superbuild/build/install/share/codyco/robots/iCubNancy01

6. bring the robot in the base retargeting position (poseLuigi)
7. while the robot is still in the air (feet in the air, not on the ground), calibrate the dynamics

yarp rpc /wholeBodyDynamics/rpc 
>>calib all 300

8. bring down the robot with the lifter. make sure the feet are parallel to the ground as much as possible.
check the icubGui for the forces at the feet to be parallel

9. increase the simmetry of the forces at the feet by using the macumba:

cd /software/src/codyco-superbuild/build/install/bin
sh twoFeetStandingIdleAndCalib.sh 

if it does not work well, try to adjust the feet pitch and roll with the yarpmotorgui

10. someone must stay close to the robot now, ready with the red button

11. launch datadumpers
12. launch retargeting module
13. open the torque controller on simulink: the robot will be torque controlled!

14. start playing the xsens data from the yarpdataplayer, or stream data from the xsens
