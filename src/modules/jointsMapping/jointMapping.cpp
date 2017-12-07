/* 
 * Author: Luigi Penco
 */
#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/PortInfo.h>
#include <iostream>
#include <Eigen/Dense>
#include <string>
using namespace std;
using namespace Eigen;
using namespace yarp::os;
const double PI = 3.141592653589793;
double offset = 0.1;

int main(int argc, char *argv[]) {
    Network yarp;    
    BufferedPort<Bottle> port;
    port.open("/xsensToRobot/joints");
    while (true) {
        cout << "waiting for input" << endl;
        Bottle *input = port.read();
        if (input!=NULL) {
	   cout << "got " << input->toString().c_str() << endl;
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

	    VectorXd jointInput(32);
	    jointInput << neck_pitch*PI/180, neck_roll*PI/180, neck_yaw*PI/180, torso_yaw*PI/180, torso_roll*PI/180, torso_pitch*PI/180, l_shoulder_pitch*PI/180, l_shoulder_roll*PI/180, l_shoulder_yaw*PI/180, l_elbow*PI/180, l_wrist_prosup*PI/180, l_wrist_pitch*PI/180, l_wrist_yaw*PI/180, r_shoulder_pitch*PI/180, r_shoulder_roll*PI/180, r_shoulder_yaw*PI/180, r_elbow*PI/180, r_wrist_prosup*PI/180, r_wrist_pitch*PI/180, r_wrist_yaw*PI/180, l_hip_pitch*PI/180, l_hip_roll*PI/180, l_hip_yaw*PI/180, l_knee*PI/180, l_ankle_pitch*PI/180, l_ankle_roll*PI/180, r_hip_pitch*PI/180, r_hip_roll*PI/180, r_hip_yaw*PI/180, r_knee*PI/180, r_ankle_pitch*PI/180, r_ankle_roll*PI/180;


	    MatrixXd jointLim(2,32);
	    jointLim << -0.51679199, -0.349066, 0.785398, -0.349066, -0.523599, -0.872665, -1.65806, 0, -0.645772, 0.261799, -1.0472, -1.392773, -0.3455752, -1.65806, 0, -0.645772, 0.261799, -1.0472, -1.392773, -0.3455752, -0.610865, -0.261799, -1.22173, -1.74533, -0.523599, -0.349066, -0.610865, -0.261799, -1.22173, -1.74533, -0.523599, -0.349066,
			0.29530971, 0.349066, 0.785398, 1.22173, 0.523599, 0.872665, 0.174533, 2.79253, 1.39626, 1.85005, 1.0472, 0.42149701, 0.43196899, 0.174533, 2.79253, 1.39626, 1.85005, 1.0472, 0.42149701, 0.43196899, 1.48353, 1.5708, 1.22173, 0, 0.523599, 0.349066, 1.48353, 1.5708, 1.22173, 0, 0.523599, 0.349066;


	    /* Edit the JointAngles according to the Joitn limits of the iCub */ 
/*	    for (int i = 0; i < jointInput.size(); i++){ 
	    	if (jointInput(i) <= jointLim(0,i))		//check min
			jointInput(i) = jointLim(0,i) + offset;
		if (jointInput(i) >= jointLim(1,i))		//check max
			jointInput(i) = jointLim(1,i) - offset;
	    }
*/
 	    neck_pitch = jointInput(0);
            neck_roll = jointInput(1); 
	    neck_yaw = jointInput(2);
	    torso_yaw = jointInput(3);
	    torso_roll = jointInput(4);
	    torso_pitch = jointInput(5);
	    l_shoulder_pitch = jointInput(6);
	    l_shoulder_roll = jointInput(7);
     	    l_shoulder_yaw = jointInput(8);
	    l_elbow = jointInput(9);
	    l_wrist_prosup = jointInput(10);
	    l_wrist_pitch = jointInput(11);
	    l_wrist_yaw = jointInput(12);
	    r_shoulder_pitch = jointInput(13);
	    r_shoulder_roll = jointInput(14);
	    r_shoulder_yaw = jointInput(15);
	    r_elbow = jointInput(16);
	    r_wrist_prosup = jointInput(17);
	    r_wrist_pitch = jointInput(18);
	    r_wrist_yaw = jointInput(19);
	    l_hip_pitch = jointInput(20);
	    l_hip_roll = jointInput(21);
	    l_hip_yaw = jointInput(22);
	    l_knee = jointInput(23);
	    l_ankle_pitch = jointInput(24);
	    l_ankle_roll = jointInput(25);
	    r_hip_pitch = jointInput(26);
	    r_hip_roll = jointInput(27);
	    r_hip_yaw = jointInput(28);
	    r_knee = jointInput(29);
	    r_ankle_pitch = jointInput(30);
	    r_ankle_roll = jointInput(31);


 	    Bottle& output = port.prepare();
            output.clear();

	    /* JointAngles coordinates to send to the iCub */
	    //output.addDouble(neck_pitch);
	    //output.addDouble(neck_roll);
	    //output.addDouble(neck_yaw);   
	    output.addDouble(torso_yaw);
	    output.addDouble(torso_roll);
	    output.addDouble(torso_pitch);
	    output.addDouble(l_shoulder_pitch);
	    output.addDouble(l_shoulder_roll);
	    output.addDouble(l_shoulder_yaw);
	    output.addDouble(l_elbow);
 	    //output.addDouble(l_wrist_prosup);
	    //output.addDouble(l_wrist_pitch);
	    //output.addDouble(l_wrist_yaw);
	    output.addDouble(r_shoulder_pitch);
	    output.addDouble(r_shoulder_roll);
	    output.addDouble(r_shoulder_yaw);
	    output.addDouble(r_elbow);
 	    //output.addDouble(r_wrist_prosup);
	    //output.addDouble(r_wrist_pitch);
	    //output.addDouble(r_wrist_yaw);
	    output.addDouble(l_hip_pitch);
	    output.addDouble(l_hip_roll);
	    output.addDouble(l_hip_yaw);
	    output.addDouble(l_knee);
	    output.addDouble(l_ankle_pitch);
	    output.addDouble(l_ankle_roll);
	    output.addDouble(r_hip_pitch);
	    output.addDouble(r_hip_roll);
	    output.addDouble(r_hip_yaw);
	    output.addDouble(r_knee);
	    output.addDouble(r_ankle_pitch);
	    output.addDouble(r_ankle_roll);
	    port.write();
        }
    }
    return 0;
}
