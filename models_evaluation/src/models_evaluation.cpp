/****************************************************************************************
 * Filename 	: models_evaluation.cpp
 * Path			: models_evaluation/src
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: source file of models_evaluation
 **************************************************************************************/
#include "models_evaluation.hpp"

/// constructor initialization
Models_evaluation::Models_evaluation()
{
	/// Specifying the path and the rigid body information
	Models_evaluation::tip_link  = "arm_link_5";
	Models_evaluation::base_link = "mobile_link";
	Models_evaluation::urdf_path = ros::package::getPath("models_evaluation");
	Models_evaluation::urdf_robocup = Models_evaluation::urdf_path + "/data/youbot.urdf";
	Models_evaluation::urdf_luhbots = Models_evaluation::urdf_path + "/data/luhbots.urdf";
	/// Initializing the Keiser's model variables
	Models_evaluation::j_pos.resize(FIVE);
	Models_evaluation::vel.resize(FIVE);
	Models_evaluation::acc.resize(FIVE);
	Models_evaluation::j_pos.setZero();
	Models_evaluation::vel.setZero();
	Models_evaluation::acc.setZero();
	/// Static parameters (model values)
		/// Building a kinematic tree 
    if (!kdl_parser::treeFromFile(Models_evaluation::urdf_robocup, Models_evaluation::tree)) {
        cout << "Failed to construct kdl tree " << endl;
        exit(1);
    }
	
	/// Obtain the kinematic chain from base_link to tip_link from the tree  
	if (!Models_evaluation::tree.getChain(Models_evaluation::base_link, Models_evaluation::tip_link, Models_evaluation::chain_robocup))
        cout << "Couldn't find chain from " << Models_evaluation::base_link.c_str() << "to " << Models_evaluation::tip_link.c_str() << endl;

	/// Luhbot's model parameters taken from params.dat
	Models_evaluation::init_params.mass_5 = 0.4127435232645671;
    Models_evaluation::init_params.mass_4 = 0.9736744251972129;
    Models_evaluation::init_params.mass_3 = 0.7065456362157415;
    Models_evaluation::init_params.mass_2 = 0.6704054434195545;
    Models_evaluation::init_params.com_radius_5 = 0.2189300979881188;
    Models_evaluation::init_params.com_radius_4 = 0.04833327364289946;
    Models_evaluation::init_params.com_radius_3 = 0.08795328939031184;
    Models_evaluation::init_params.com_radius_2 = 0.06402917528867183;
    Models_evaluation::init_params.com_angle_5 = -0.1850041679383009;
    Models_evaluation::init_params.com_angle_4 = 0.002036069407585526;
    Models_evaluation::init_params.com_angle_3 = 0.1307947824954063;
    Models_evaluation::init_params.com_angle_2 = 0.2964549495469563;
    Models_evaluation::init_params.friction_2 = 0.0645933093552246;
    Models_evaluation::init_params.friction_3 = -0.04209267737562461;
    Models_evaluation::init_params.friction_4 = 0.03939221159704069;
	Models_evaluation::init_params.friction_5 = 0.313249255481345;
	Models_evaluation::init_params.gravity = 9.81;
}
	
/****************************************************************************************
 * Functionname		: robocupModel
 * Input			: Joint angles(q_in), velocities(qdot_in), accelerations(qdotdot_in)
 * Output			: outClientFile, outJT for joint torques
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Torques calculation on each joint based on Robocup-at-work's
 * 					  model
 * Return 			: vector with joint torques
 **************************************************************************************/
std::vector<double> Models_evaluation::robocupModel(JntArray q_in, JntArray qdot_in, JntArray qdotdot_in, int in, ofstream &outClientFile, ofstream &outJt)
{

	/// Initializing the joint array variables 
	Models_evaluation::f.resize(Models_evaluation::chain_robocup.getNrOfSegments());
    Models_evaluation::tau.resize(Models_evaluation::chain_robocup.getNrOfJoints());

	/// Forward kinematics function declaration  
	ChainFkSolverPos_recursive fksolver(Models_evaluation::chain_robocup);
	/// Inverse dynamics solver declaration
	ChainIdSolver_RNE idsolver(Models_evaluation::chain_robocup,Vector(0,0 ,GRAV));
	 KDL::Wrenches wrenchNull_r(chain_robocup.getNrOfSegments(), KDL::Wrench::Zero());
	/// Convert joint coordinates to cartesian coordinates
	Models_evaluation::kinematics_status = fksolver.JntToCart(q_in,Models_evaluation::T);
	if(Models_evaluation::kinematics_status < ZERO)
		cout << "Error: could not calculate forward kinematics for robocup model" << endl;
	
	/// Convert Cartesian coordinates to joint torques
	idsolver.CartToJnt(q_in,qdot_in,qdotdot_in,wrenchNull_r,Models_evaluation::tau);
	/// Store the obtained torque values in the output file
	outClientFile << "\nExperiment no. : " << in << endl;
	outClientFile << "\nRobocup-at-work torques :" << endl;
	for(unsigned int j=ZERO;j<Models_evaluation::chain_robocup.getNrOfJoints();j++)
	{
		outClientFile << setiosflags(ios::left) << setiosflags(ios::fixed | ios::showpoint) << "J" << j+ONE << " :" << Models_evaluation::tau(j) <<  endl;
		outJt << Models_evaluation::tau(j) << " ";
	}
	/// Store the end-effector frame values in the output file for the reference
	outClientFile << "Robocup frame :"<< endl << Models_evaluation::T << endl;
	/// Vector to store the torque values
	std::vector<double> output;
	output.resize(5);
	output[0] =  Models_evaluation::tau(0);
	output[1] =  Models_evaluation::tau(1);
	output[2] =  Models_evaluation::tau(2);
	output[3] =  Models_evaluation::tau(3);
	output[4] =  Models_evaluation::tau(4);
	
	outJt << endl;
	/// Returning the calculated torques 
	return output;
}

/****************************************************************************************
 * Functionname		: keiserModel
 * Input			: Joint angles(q_in), velocities(qdot_in), accelerations(qdotdot_in)
 * Output			: outClientFile
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Torques are calculated for each joint based on Keiser's
 * 					  model.
 **************************************************************************************/
std::vector<double> Models_evaluation::keiserModel(JntArray q_in, JntArray qdot_in, JntArray qdotdot_in, int in, ofstream &outClientFile)
{
	/// Initializing the joint array variables 
	Models_evaluation::j_pos(ZERO) = q_in(ZERO);
	Models_evaluation::j_pos(ONE) = q_in(ONE);
	Models_evaluation::j_pos(TWO) = q_in(TWO);
	Models_evaluation::j_pos(THREE) = q_in(THREE);
	Models_evaluation::j_pos(FOUR) = q_in(FOUR);
	Models_evaluation::vel(ZERO) = qdot_in(ZERO);
	Models_evaluation::vel(ONE) = qdot_in(ONE);
	Models_evaluation::vel(TWO) = qdot_in(TWO);
	Models_evaluation::vel(THREE) = qdot_in(THREE);
	Models_evaluation::vel(FOUR) = qdot_in(FOUR);
	Models_evaluation::acc(ZERO) = qdotdot_in(ZERO);
	Models_evaluation::acc(ONE) = qdotdot_in(ONE);
	Models_evaluation::acc(TWO) = qdotdot_in(TWO);
	Models_evaluation::acc(THREE) = qdotdot_in(THREE);
	Models_evaluation::acc(FOUR) = qdotdot_in(FOUR);
	
	/// IK solver assumes the Offset values for the home position
	Models_evaluation::j_pos = youbot2torque(Models_evaluation::j_pos);
	/// Calculate torques based on the given joint angles, velocities and accelerations
	calcTorques(Models_evaluation::j_pos, Models_evaluation::vel, Models_evaluation::acc, Models_evaluation::torques);
	/// Get the transformation matrix after the forward kinematics calculation
	getCartPos(Models_evaluation::j_pos, Models_evaluation::cart_pos);
	/// Reversing the sign of the torque for joint 2 since the movement is in negative direction
	/// Models_evaluation::torques(2) = -Models_evaluation::torques(2);
	/// Store torques values in the output file
	outClientFile << "Keiser's Torques :" << endl;	
	for(unsigned int j=ZERO;j<FIVE;j++)
	{
		outClientFile << setiosflags(ios::left) << setiosflags(ios::fixed | ios::showpoint) << "J" << j+ONE << " :"<< Models_evaluation::torques(j) <<  endl;
	}	
	/// Store the end-effector frame to the output file
	outClientFile << "Keiser's frame :" << endl;
	for(unsigned int k=ZERO;k<FOUR;k++)
	{
		for(unsigned int l=ZERO;l<FOUR;l++)
		{
			outClientFile << Models_evaluation::cart_pos(k,l) << " ";
		}
		outClientFile << endl;
	}
	/// Vector to store the torque values
	std::vector<double> output;
	output.resize(5);
	output[0] = Models_evaluation::torques(0);
	output[1] = Models_evaluation::torques(1);
	output[2] = -Models_evaluation::torques(2);
	output[3] = Models_evaluation::torques(3);
	output[4] = Models_evaluation::torques(4);
	/// Returning the calculated torque 
	return output;
}

/****************************************************************************************
 * Functionname		: luhbotsModel
 * Input			: Joint angles(q_in), velocities(qdot_in), accelerations(qdotdot_in)
 * Output			: outClientFile 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Torques are calculated for each joint based on Luhbot's
 * 					  model
 * Return 			: vector with joint torques
 **************************************************************************************/
std::vector<double> Models_evaluation::luhbotsModel(JntArray q_in, JntArray qdot_in, JntArray qdotdot_in, int in, ofstream &outClientFile)
{
	/// Initializing the joint angles variable
	Models_evaluation::pos.setQ1(q_in(0));
	Models_evaluation::pos.setQ2(q_in(1));
	Models_evaluation::pos.setQ3(q_in(2));
	Models_evaluation::pos.setQ4(q_in(3));
	Models_evaluation::pos.setQ5(q_in(4));

	/// To set the dynamic properties for the getStaticJointEffort calculation
	Models_evaluation::dyn.setStaticParameters(Models_evaluation::init_params);
	/// To calculate the static joint effort based on luhbot's model
	Models_evaluation::eff=Models_evaluation::dyn.getStaticJointEffort(Models_evaluation::pos);
	/// Store torques in the output file
	outClientFile << "Luhbot's model torques \n";
	outClientFile << "J1: " << Models_evaluation::eff.q1() << "\nJ2: " << Models_evaluation::eff.q2() << "\nJ3: " << Models_evaluation::eff.q3() << "\nJ4: " << Models_evaluation::eff.q4() << "\nJ5: " << Models_evaluation::eff.q5() << endl;
	/// Vector to store the torque values
	std::vector<double> vec;
	vec.resize(5);
	vec[0] = Models_evaluation::eff.q1();
	vec[1] = Models_evaluation::eff.q2();
	vec[2] = Models_evaluation::eff.q3();
	vec[3] = Models_evaluation::eff.q4();
	vec[4] = Models_evaluation::eff.q5();
	/// Returning the calculated torque 
	return vec;
}
