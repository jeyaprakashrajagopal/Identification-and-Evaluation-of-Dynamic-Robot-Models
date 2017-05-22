/**************************************************************************************
 * Filename 	: gravitycompensation.hpp
 * Path			: models_evaluation/include/models_evaluation
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: Header file for gravity compensation
 **************************************************************************************/
#ifndef __GRAVITY_COMPENSATION_
#define __GRAVITY_COMPENSATION_
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <ros/package.h>
#include <kdl/frames_io.hpp>
#include "models_evaluation.hpp"

/// Namespace usage
using namespace std;
using namespace ros;
using namespace KDL;

/**
 * @brief Base class for gravity compensation.
 */
class Gravitycompensation
{
	public:
		/**
		 * @brief Constructor declaration.
		*/
		Gravitycompensation();
		/**
		 * @brief Limiting torques to avoid the damage to the gearbox
		 * @param torques.
		 * @return Torque output in a file, vector of torques
		*/
		bool limitTorques(KDL::JntArray tor);
		/**
		 * @brief Gravity compensation based on KDL
		 * @param Joint angles, velocities, accelerations.
		 * @return Limited torques
		*/
		bool gravityCompensation(KDL::JntArray q, KDL::JntArray qdot, KDL::JntArray qddot);

		double maxTorque[5];
		Models_evaluation idTrq;
};
#endif
