/**
* Copyright: (C) 2017 Inria
* Author: Luigi Penco
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

// Local includes
#include "qpOASES.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <Eigen/Cholesky>
#include "LIPThread.h"
#include <math.h>   

using namespace yarp::os;
using namespace std;
using namespace Eigen;

const double G = 9.80665;


//===============================================
//         Main THREAD
//===============================================

LIPThread::LIPThread(string _name,
			       string _robotName,
			       int _period,
			       Eigen::Vector2d m_lambdaD)
:   RateThread(_period),
	moduleName(_name),
	robotName(_robotName),
	lambdaD(m_lambdaD),
	printCountdown(0),
	printPeriod(2000)

{

yInfo() << "Launching LIPThread with name : " << _name << " and robotName " << _robotName << " and period " << _period;

}


bool LIPThread::threadInit()
{  
	//opening ports
	com_port.open(string("/comStabilizer/LIPdata").c_str());

	qpdata.resize(18);
	yInfo() << "LIPThread::threadInit finished successfully.";

	return true;
}


void LIPThread::publishStableCoM()
{
	Bottle& output = com_port.prepare();
	output.clear();

	for (int i=0; i<2; i++){
		output.addDouble(comPos_stab(i));
	}
	for (int i=0; i<2; i++){
		output.addDouble(comVel_stab(i));
	}
	for (int i=0; i<2; i++){
		output.addDouble(zmp(i));
	}
	for (int i=0; i<2; i++){
		output.addDouble(zmp_actcom(i));
	}
	com_port.write();
}


void LIPThread::getQPdata()
{
	Bottle *input = com_port.read(false);
	if(input!=NULL){   
		streamingCoM = true;
		for (int i=0; i<qpdata.size(); i++){
			qpdata(i) = input->get(i).asDouble();
		}
		for (int i=0; i<2; i++){
			comPos_des(i) = qpdata(i);
			comVel_des(i) = qpdata(i+2);
			comAcc_des(i) = qpdata(i+4);
			comPos(i) = qpdata(i+6);
			comVel(i) = qpdata(i+8);
			comAcc(i) = qpdata(i+10);
			bConstraintMin(i) = qpdata(i+12);
			bConstraintMax(i) = qpdata(i+14);
		}
		hc = qpdata(16);
		T = qpdata(17);
		zmp_actcom = -(hc/G)*comAcc + comPos;
	}
	else{
		streamingCoM = false;
	}
}


//------ RUN ------
void LIPThread::run()
{
	getQPdata();
	LIPretarget();
	double    t = yarp::os::Time::now();
	publishData();
}


//----------------------------
//-- Robot-LIP CoM Retargeting
//----------------------------
void LIPThread::LIPretarget() {
	// Solve QP
    zmp(0) = solveQPX();
    zmp(1) = solveQPY();
    // Get the CoM velocity value associated to the zmp solution
    comVel_stab(0) = comVel(0) + (T*G/hc)*comPos(0) - (T*G/hc)*zmp(0);
    comVel_stab(1) = comVel(1) + (T*G/hc)*comPos_des(1) - (T*G/hc)*zmp(1);
    // integrate to get the position
    comPos_stab(0) = comPos(0) + T*comVel_stab(0);
    comPos_stab(1) = comPos(1) + T*comVel_stab(1);
}

// Eigen::VectorXd LIPThread::solveQP() {

// 	USING_NAMESPACE_QPOASES 

// 	int nVariables = 2;
// 	int nConstraints = 2;

// 	qpOASES::real_t H[2*2] = { T*T*G*G/(hc*hc) + lambdaD(0), 0.0, 0.0, T*T*G*G/(hc*hc) + lambdaD(1) };
// 	qpOASES::real_t g[2] = { 2*comVel_des(0)*T*G/hc - 2*comVel(0)*T*G/hc - 2*comPos(0)*T*T*G*G/(hc*hc), 2*comVel_des(1)*T*G/hc - 2*comVel(1)*T*G/hc - 2*comPos(1)*T*T*G*G/(hc*hc) };

// 	qpOASES::real_t lb[nConstraints];
// 	qpOASES::real_t ub[nConstraints];

// 	for(int i=0; i<nConstraints; i++){
// 		lb[i] = bConstraintMin(i);
// 		ub[i] = bConstraintMax(i);
// 	}

// 	qpOASES::real_t xOpt[nVariables];

// 	// Quadratic problem
//     qpOASES::QProblemB qp(nVariables);

// 	qpOASES::Options options;
// 	//options.enableFlippingBounds = BT_FALSE;
// 	options.initialStatusBounds = ST_INACTIVE;
// 	options.numRefinementSteps = 1;
// 	options.enableCholeskyRefactorisation = 1;
// 	qp.setOptions(options);

// 	qpOASES::int_t nWSR = 10;
// 	qp.init( H,g,lb,ub, nWSR,0 );

// 	qp.getPrimalSolution(xOpt);

// 	Eigen::VectorXd decisionVariables(3);

// 	for(int i=0; i<2; i++){
// 		decisionVariables(i) = xOpt[i];
// 	}

// 	qpOASES::real_t fit = qp.getObjVal();
// 	decisionVariables(2) = fit;


// 	return decisionVariables;
// }

double LIPThread::solveQPX() {

	USING_NAMESPACE_QPOASES 

	qpOASES::real_t H[1]= { T*T*G*G/(hc*hc) + lambdaD(0) };
	qpOASES::real_t g[1] = { 2*comVel_des(0)*T*G/hc - 2*comVel(0)*T*G/hc - 2*comPos(0)*T*T*G*G/(hc*hc) };

	qpOASES::real_t lb[1];
	qpOASES::real_t ub[1];

	lb[0] = bConstraintMin(0);
	ub[0] = bConstraintMax(0);

	qpOASES::real_t xOpt[1];

	// Quadratic problem
    qpOASES::QProblemB qp(1);

	qpOASES::Options options;
	//options.enableFlippingBounds = BT_FALSE;
	options.initialStatusBounds = ST_INACTIVE;
	options.numRefinementSteps = 1;
	options.enableCholeskyRefactorisation = 1;
	qp.setOptions(options);

	qpOASES::int_t nWSR = 10;
	qp.init( H,g,lb,ub, nWSR,0 );

	qp.getPrimalSolution(xOpt);

	double decisionVariable;

	decisionVariable = xOpt[0];

	return decisionVariable;
}

double LIPThread::solveQPY() {

	USING_NAMESPACE_QPOASES 

	qpOASES::real_t H[1]= { T*T*G*G/(hc*hc) + lambdaD(1) };
	qpOASES::real_t g[1] = { 2*comVel_des(1)*T*G/hc - 2*comVel(1)*T*G/hc - 2*comPos(1)*T*T*G*G/(hc*hc) };

	qpOASES::real_t lb[1];
	qpOASES::real_t ub[1];

	lb[0] = bConstraintMin(1);
	ub[0] = bConstraintMax(1);

	qpOASES::real_t yOpt[1];

	// Quadratic problem
    qpOASES::QProblemB qp(1);

	qpOASES::Options options;
	//options.enableFlippingBounds = BT_FALSE;
	options.initialStatusBounds = ST_INACTIVE;
	options.numRefinementSteps = 1;
	options.enableCholeskyRefactorisation = 1;
	qp.setOptions(options);

	qpOASES::int_t nWSR = 10;
	qp.init( H,g,lb,ub, nWSR,0 );

	qp.getPrimalSolution(yOpt);

	double decisionVariable;

	decisionVariable = yOpt[0];

	return decisionVariable;
}


void LIPThread::publishData()
{
	if (streamingCoM){
		publishStableCoM();
	}

	printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
	if( printCountdown == 0 ) {
		double avgTime, stdDev, avgTimeUsed, stdDevUsed;
		getEstPeriod(avgTime, stdDev);
		getEstUsed(avgTimeUsed, stdDevUsed);
	}
}


void LIPThread::closePort()
{
	yInfo() << "Closing com stabilizer port";
	com_port.interrupt();
	com_port.close();
	yInfo() << "com stabilizer closed";
}
