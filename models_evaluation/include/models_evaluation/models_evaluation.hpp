/****************************************************************************************
 * Filename 	: models_evaluation.hpp
 * Path			: models_evaluation/include/models_evaluation
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: Header file for models_evaluation
 **************************************************************************************/
#ifndef __MODELS_EVALUATION_
#define __MODELS_EVALUATION_
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include "kdl/chaindynparam.hpp"
#include <youbot_arm_dynamics_symbolic.h>
#include <youbot_arm_forward_kinematics.h>
#include <luh_youbot_kinematics/arm_kinematics.h>
#include <luh_youbot_kinematics/arm_dynamics.h>
#include <luh_youbot_msgs/JointVector.h>

/// Namespace usage
using namespace std;
using namespace ros;
using namespace KDL;
using namespace urdf;
using namespace luh_youbot_kinematics;
/// Macro to convert degrees to radians
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
/// Macro to convert radians to degrees
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)
/// Using the macros instead of numbers in the code
#define ZERO 0.0
#define ONE 1
#define TWO 2
#define THREE 3
#define FOUR 4
#define FIVE 5
/// gravity acceleration
#define GRAV -9.81
/**
 * @brief Base class for models evaluation.
 */
class Models_evaluation
{
	public:
		/**
		 * @brief Constructor declaration.
		*/
		Models_evaluation();

		/**
		 * @brief Calculates the torques based on the Robocup's model.
		 * @param Joint angles, velocities, accelerations.
		 * @return Torque and end-effector frame output in a file, vector of torques
		*/
		std::vector<double> robocupModel(JntArray, JntArray, JntArray, int, ofstream &, ofstream &);
		/**
		 * @brief Calculates the torques based on the Keiser's model.
		 * @param Joint angles, velocities, accelerations.
		 * @return Torque and end-effector frame output in a file, vector of torques
		*/
		std::vector<double> keiserModel(JntArray, JntArray, JntArray, int, ofstream &);
		/**
		 * @brief Calculates the torques based on the Luhbot's model.
		 * @param Joint angles, velocities, accelerations.
		 * @return Torque output in a file, vector of torques
		*/
		std::vector<double> luhbotsModel(JntArray, JntArray, JntArray, int, ofstream &);

		/// Joint array for torques declarations
		JntArray tau;
		/// Kinematic chain variables declaration
		Chain chain_robocup, chain_luhbots;
		/// End-effector frame for both Robocup-at-work and Luhbot's
		Frame T, T_luh;
		/// Wrenches declaration for the 
		Wrenches f,f_luh;
		/// Kinematic tree declarationfor for Robocup-at-work model
		Tree tree, luhbots_tree;
		/// Strings used to specify the URDF path
		string urdf_robocup, urdf_luhbots,base_link, tip_link, urdf_path;
		/// To specify the total number of experiments being conducted
		int linenum;
		/// Joint angles, velocities, accelerations, torques variable declarations for Keiser's model
		Eigen::VectorXd j_pos,torques,vel,acc; 
		/// Variable declaration for Keiser's end-effector frame matrix
		Eigen::Affine3d cart_pos;		
		/// To check the forward kinematics is successful or not
		bool kinematics_status;
		/// Joint position declaration
		JointPosition pos;
		/// Static paramaters with the model
		StaticParameters init_params;
		/// Object declaration for Luhbot's dynamics
		YoubotArmDynamics dyn;
		/// Object delcaration for JointEffort vector for luhbots model
		JointEffort eff;
};
#endif
