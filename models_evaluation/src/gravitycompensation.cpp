/****************************************************************************************
 * Filename 	: models_evaluation.cpp
 * Path			: models_evaluation/src
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: source file of models_evaluation package
 **************************************************************************************/
#include "gravitycompensation.hpp"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <youbot_driver/testing/YouBotArmTest.hpp>

using namespace std;
using namespace youbot;

float joint_pos_global[5], joint_vel_global[5], joint_eff_global[5]; 
unsigned int pos,vel,eff;

bool Gravitycompensation::gravityCompensation(KDL::JntArray q, KDL::JntArray qdot, KDL::JntArray qddot)
{
	ofstream outTrq;
	bool chkTrq = false;
	KDL::JntArray outTrqJntArray;
	vector<double> trqFromModel;
	trqFromModel.resize(5);
	outTrq.open("gravityCompensation.dat", ios::out);
	
	trqFromModel = idTrq.keiserModel(q, qdot, qddot, 1, outTrq);
	for(unsigned int in = 0; in < 5;in++)
		outTrqJntArray(in) = trqFromModel[in];
		
	chkTrq = Gravitycompensation::limitTorques(outTrqJntArray);
	if(chkTrq == true)
		return true;
	else
		return false;
}

bool Gravitycompensation::limitTorques(KDL::JntArray tor)
{
	for(unsigned int jntNo = 0; jntNo < 5; jntNo++)
	{
		if(tor(jntNo) > maxTorque[jntNo])
			tor(jntNo) = maxTorque[jntNo];
		else if(tor(jntNo) < -maxTorque[jntNo])
			tor(jntNo) = -maxTorque[jntNo];
	}
	return true;
}