/****************************************************************************************
 * Filename 	: ModelOptimizer.cpp
 * Path			: models_evaluation/src
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: source file for optimizing the model based on robot experiments
 **************************************************************************************/
#include "ModelOptimizer.hpp"

Models_evaluation eval;

ModelOptimizer modelOpti;
/// Counts the number of iterations of the objective function
unsigned long iteration_count = 0, iteration_count_dyn=0;
/// Object to hold an unknown smallest error value
double min_error = HUGE_VAL;
/// Objects to hold the initial and final output params 
ModelOptimizer::StaticParameters best_params;
ModelOptimizer::StaticParameters param_data;
ModelOptimizer::DynamicParameters best_params_dyn;
ModelOptimizer::DynamicParameters param_data_dyn;
/// Number of trajectories 
unsigned int poses; 
/// Checking the mode of configuration static or moving
string config;
/// Object of a struct to hold the robot's experiment data
ModelOptimizer::State *data_new;
/// Empty constructor declaration	
ModelOptimizer::ModelOptimizer(){}
/// to get the catkin package path from ros 
string packagePath = ros::package::getPath("models_evaluation");
string filepath = packagePath + "/data/outF.dat";
const char *out_filePath = filepath.c_str();
ofstream outFrame(out_filePath, ios::out);
ofstream err_id("/home/jp/err.dat", ios::out);

/// Degrees to radian conversion
double d2r(double v) {
	return v / 180 * M_PI;
}

/****************************************************************************************
 * Functionname		: robocupModel
 * Input			: -
 * Output			: Model stored in a file
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Function to save the model after the identification process in both 
 * 				      static and moving configurations
 **************************************************************************************/
void ModelOptimizer::saveParams()
{
	std::ofstream outClientFile;
	string outParamPath, outparam;
	if(config[0] == 's')
	{
		outparam = packagePath + "/data/outparam_sta.dat";	
		const char *out_filePath = outparam.c_str();
		
		outClientFile.open(out_filePath, ios::out);
		
		for(unsigned int i= 0; i< 4;i++)
			outClientFile << "mass" << i+2 << " " <<param_data.mass[i] << endl;
		for(unsigned int i=0;i<4;i++)
			outClientFile << "com_r" << i+2 << " "  << param_data.com_radius[i] << endl;
		for(unsigned int i=0;i<4;i++)
			outClientFile << "com_a" << i+2 << " "  << param_data.com_angle[i] << endl;
		outClientFile<< endl;
	}
	else if(config[0] == 'm')
	{
		outparam = packagePath + "/data/outparam_dyn.dat";
		const char *out_filePath = outparam.c_str();
		
		outClientFile.open(out_filePath, ios::out);
		
		for(unsigned int i= 0; i< 4;i++)
			outClientFile << "mass" << i+2 << " " <<param_data_dyn.mass[i] << endl;
		for(unsigned int i=0;i<4;i++)
			outClientFile << "com_r" << i+2 << " "  << param_data_dyn.com_radius[i] << endl;
		for(unsigned int i=0;i<4;i++)
			outClientFile << "com_a" << i+2 << " "  << param_data_dyn.com_angle[i] << endl;
		outClientFile<< endl;
	}
	outClientFile.close();
}

/****************************************************************************************
 * Functionname		: getDynamicEffort
 * Input			: Joint angles(q), velocities(qdot), accelerations(qdotdot)
 * Output			: Model stored in a file
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Function to compute the dynamic effort of both the 
 *                    moving configurations using KDL library.
 * Return 			: vector with joint torques
 **************************************************************************************/
std::vector<double> ModelOptimizer::getDynamicEffort(std::vector<double> q,std::vector<double> qdot,std::vector<double> qddot)
{
	/// chain declaration
    KDL::Chain chain;
    /// input joint arrays declaration
	JntArray tau, q_in,qdot_in,qdotdot_in;
	q_in.resize(5);
	qdot_in.resize(5);
	qdotdot_in.resize(5);

	q_in(0) = q[0];
	q_in(1) = q[1];
	q_in(2) = q[2];
	q_in(3) = q[3];
	q_in(4) = q[4];	
	
	qdot_in(0) = qdot[0];
	qdot_in(1) = qdot[1];
	qdot_in(2) = qdot[2];
	qdot_in(3) = qdot[3];
	qdot_in(4) = qdot[4];	

	qdotdot_in(0) = qddot[0];
	qdotdot_in(1) = qddot[1];
	qdotdot_in(2) = qddot[2];
	qdotdot_in(3) = qddot[3];
	qdotdot_in(4) = qddot[4];	
	/// To compute the COM in cartesian space based on the polar coordinates used in the optimization
	double x = param_data_dyn.com_radius[0]*cos(param_data_dyn.com_angle[0]), y = param_data_dyn.com_radius[0]*sin(param_data_dyn.com_angle[0]);
	double x1 = param_data_dyn.com_radius[1]*cos(param_data_dyn.com_angle[1]), y1 = param_data_dyn.com_radius[1]*sin(param_data_dyn.com_angle[1]);
	double x2 = param_data_dyn.com_radius[2]*cos(param_data_dyn.com_angle[2]), y2 = param_data_dyn.com_radius[2]*sin(param_data_dyn.com_angle[2]);
	double x3 = param_data_dyn.com_radius[3]*cos(param_data_dyn.com_angle[3]), y3 = param_data_dyn.com_radius[3]*sin(param_data_dyn.com_angle[3]);	
	/// Rotational inertia of the rigid body
	RotationalInertia rotInertia(0.01,0.01,0.01,0.0,0.0,0.0);
	/// Rigidbody interial properties including mass, COM, rotational inertia
	RigidBodyInertia rigidJt1(1.39, Vector(0,0,0.0));
	RigidBodyInertia rigidJt2(param_data_dyn.mass[0], Vector(x,y, param_data_dyn.com_z[0]),rotInertia);
	RigidBodyInertia rigidJt3(param_data_dyn.mass[1], Vector(x1,y1, param_data_dyn.com_z[1]),rotInertia);
	RigidBodyInertia rigidJt4(param_data_dyn.mass[2], Vector(x2,y2, param_data_dyn.com_z[2]),rotInertia);
	RigidBodyInertia rigidJt5(param_data_dyn.mass[3], Vector(x3,y3, param_data_dyn.com_z[3]),rotInertia);
	/// To count the offset in joints
	double offset[5] = {d2r(-169),d2r(-65),d2r(151),d2r(-102.5),d2r(-167.5)};
	/// This code is reproduced from https://github.com/mfueller/RobotManipulation_WS2013/blob/master/src/kdl_kinematic_solver_test.cpp
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  +M_PI_2,  0.000, offset[0] + M_PI),rigidJt1));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000, offset[1] - M_PI_2),rigidJt2));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000, offset[2]),rigidJt3));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.081,  0,       0.000, offset[3]),rigidJt4));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.000, -M_PI_2,  0.000, offset[4] + M_PI_2),rigidJt5));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, 0, -0.137, 0)));
	/// Declaration of wrenches for the calculation in kdl
	KDL::Wrenches f;
	tau.resize(chain.getNrOfJoints());
	/// Forward kinematic solver declaration
    ChainFkSolverPos_recursive fksolver(chain);
	/// To obtain the frame information from the joints
    Frame T;
	f.resize(chain.getNrOfSegments());
	/// To check the status of the forward kinematic solver results
	bool kinematics_status = fksolver.JntToCart(q_in,T);
    if(kinematics_status < ZERO)
		cout << "Error: could not calculate forward kinematics for robocup model" << endl;
	outFrame << " Frame " << endl;
	outFrame << T << endl;
	/// Inverse dynamics solver declaration
	ChainIdSolver_RNE idsolver(chain,Vector(0,0,GRAV));
	/// Convert Cartesian coordinates to joint torques
	idsolver.CartToJnt(q_in,qdot_in,qdotdot_in,f,tau);
	/// Vector declaration to store the torque outputs
	std::vector<double> vec;
	vec.resize(5);
	vec[0] = tau(0);
	vec[1] = tau(1);
	vec[2] = tau(2);
	vec[3] = tau(3);
	vec[4] = tau(4);
	/// Vector is returned with torques
	return vec;
}

/****************************************************************************************
 * Functionname		: getStaticEffort
 * Input			: Joint angles(q), velocities(qdot), accelerations(qdotdot)
 * Output			: Model stored in a file
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Function to compute the dynamic effort of both the 
 *                    static configurations using KDL library.
 * Return 			: vector with joint torques
 **************************************************************************************/
std::vector<double> ModelOptimizer::getStaticEffort(std::vector<double> q,std::vector<double> qdot,std::vector<double> qddot)
{
	/// chain declaration
    KDL::Chain chain;
    /// input joint arrays declaration
	JntArray tau, q_in,qdot_in,qdotdot_in;
	q_in.resize(5);
	qdot_in.resize(5);
	qdotdot_in.resize(5);

	q_in(0) = q[0];
	q_in(1) = q[1];
	q_in(2) = q[2];
	q_in(3) = q[3];
	q_in(4) = q[4];	
	
	qdot_in(0) = qdot[0];
	qdot_in(1) = qdot[1];
	qdot_in(2) = qdot[2];
	qdot_in(3) = qdot[3];
	qdot_in(4) = qdot[4];	

	qdotdot_in(0) = qddot[0];
	qdotdot_in(1) = qddot[1];
	qdotdot_in(2) = qddot[2];
	qdotdot_in(3) = qddot[3];
	qdotdot_in(4) = qddot[4];	
	/// To compute the COM in cartesian space based on the polar coordinates used in the optimization
	double x = param_data.com_radius[0]*cos(param_data.com_angle[0]), y = param_data.com_radius[0]*sin(param_data.com_angle[0]);
	double x1 = param_data.com_radius[1]*cos(param_data.com_angle[1]), y1 = param_data.com_radius[1]*sin(param_data.com_angle[1]);
	double x2 = param_data.com_radius[2]*cos(param_data.com_angle[2]), y2 = param_data.com_radius[2]*sin(param_data.com_angle[2]);
	double x3 = param_data.com_radius[3]*cos(param_data.com_angle[3]), y3 = param_data.com_radius[3]*sin(param_data.com_angle[3]);	
	/// Rotational inertia of the rigid body
	RotationalInertia rotInertia(0.01,0.01,0.01,0.0,0.0,0.0);
	/// Rigidbody interial properties including mass, COM, rotational inertia
	RigidBodyInertia rigidJt1(1.39, Vector(0,0,0.0));
	RigidBodyInertia rigidJt2(param_data.mass[0], Vector(x,y, 0.0),rotInertia);
	RigidBodyInertia rigidJt3(param_data.mass[1], Vector(x1,y1, 0.0),rotInertia);
	RigidBodyInertia rigidJt4(param_data.mass[2], Vector(x2,y2, 0.0),rotInertia);
	RigidBodyInertia rigidJt5(param_data.mass[3], Vector(x3,y3, 0.0),rotInertia);
	/// To count the offset in joints
	double offset[5] = {d2r(-169),d2r(-65),d2r(151),d2r(-102.5),d2r(-167.5)};
	/// This code is reproduced from https://github.com/mfueller/RobotManipulation_WS2013/blob/master/src/kdl_kinematic_solver_test.cpp
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  +M_PI_2,  0.000, offset[0] + M_PI),rigidJt1));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000, offset[1] - M_PI_2),rigidJt2));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000, offset[2]),rigidJt3));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.081,  0,       0.000, offset[3]),rigidJt4));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.000, -M_PI_2,  0.000, offset[4] + M_PI_2),rigidJt5));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, 0, -0.137, 0)));
	/// Declaration of wrenches for the calculation in kdl
	KDL::Wrenches f;
	tau.resize(chain.getNrOfJoints());
	/// Forward kinematic solver declaration
    ChainFkSolverPos_recursive fksolver(chain);
	/// To obtain the frame information from the joints
    Frame T;
	f.resize(chain.getNrOfSegments());
	/// To check the status of the forward kinematic solver results
	bool kinematics_status = fksolver.JntToCart(q_in,T);
    if(kinematics_status < ZERO)
		cout << "Error: could not calculate forward kinematics for robocup model" << endl;
	outFrame << " Frame " << endl;
	outFrame << T << endl;
	/// Inverse dynamics solver declaration
	ChainIdSolver_RNE idsolver(chain,Vector(0,0,GRAV));
	/// Convert Cartesian coordinates to joint torques
	idsolver.CartToJnt(q_in,qdot_in,qdotdot_in,f,tau);
	/// Vector declaration to store the torque outputs
	std::vector<double> vec;
	vec.resize(5);
	vec[0] = tau(0);
	vec[1] = tau(1);
	vec[2] = tau(2);
	vec[3] = tau(3);
	vec[4] = tau(4);
	/// Vector is returned with torques
	return vec;
}


/****************************************************************************************
 * Functionname		: deserializeDynamicParameters
 * Input			: Vector of parameter values, DynamicParameters object   
 * Output			: Returns the struct params with the assigned values from vector 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Assigns the vector values to the struct object. The optimizer
 * 					  handles only the vector of variables. The processing
 * 					  happens in the structure for the calculations. 
 **************************************************************************************/
void deserializeDynamicParameters(const std::vector<double> &x, ModelOptimizer::DynamicParameters &params)
{
    params.mass[0]         = x[0];
    params.mass[1]         = x[1];
    params.mass[2]         = x[2];
    params.mass[3]         = x[3];
    params.com_radius[0]         = x[4];
    params.com_radius[1]         = x[5];
    params.com_radius[2]         = x[6];
    params.com_radius[3]         = x[7];
    params.com_angle[0]         = x[8];
    params.com_angle[1]         = x[9];
    params.com_angle[2]         = x[10];
    params.com_angle[3]         = x[11];
    params.com_z[0]         = x[12];
    params.com_z[1]         = x[13];
    params.com_z[2]         = x[14];
    params.com_z[3]         = x[15];
}

/****************************************************************************************
 * Functionname		: serializeDynamicParameters
 * Input			: Vector of parameter values, DynamicParameters object   
 * Output			: Returns the vector with the assigned values of struct object 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Assigns the structure values to the vector. The optimizer
 * 					  handles only the vector of variables, it is important to 
 * 					  convert the struct param values in a vector format. 
 **************************************************************************************/
void serializeDynamicParameters(const ModelOptimizer::DynamicParameters &params, std::vector<double> &x)
{
	x.resize(16);
    x[0] = params.mass[0];
    x[1] = params.mass[1];
    x[2] = params.mass[2];
    x[3] = params.mass[3];
    x[4] = params.com_radius[0];
    x[5] = params.com_radius[1];
    x[6] = params.com_radius[2];
    x[7] = params.com_radius[3];
    x[8] = params.com_angle[0];
    x[9] = params.com_angle[1];
    x[10] = params.com_angle[2];
    x[11] = params.com_angle[3];
    x[12] = params.com_z[0];
    x[13] = params.com_z[1];
    x[14] = params.com_z[2];
    x[15] = params.com_z[3];
}

/****************************************************************************************
 * Functionname		: deserializeStaticParameters
 * Input			: Vector of parameter values, StaticParameters object   
 * Output			: Returns the struct params with the assigned values from vector 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Assigns the vector values to the structure. The optimizer
 * 					  handles only the vector of variables. The processing
 * 					  happens in the structure for the calculations. 
 **************************************************************************************/
void deserializeStaticParameters(const std::vector<double> &x, ModelOptimizer::StaticParameters &params)
{
    params.mass[0]         = x[0];
    params.mass[1]         = x[1];
    params.mass[2]         = x[2];
    params.mass[3]         = x[3];
    params.com_radius[0]         = x[4];
    params.com_radius[1]         = x[5];
    params.com_radius[2]         = x[6];
    params.com_radius[3]         = x[7];
    params.com_angle[0]         = x[8];
    params.com_angle[1]         = x[9];
    params.com_angle[2]         = x[10];
    params.com_angle[3]         = x[11];
}

/****************************************************************************************
 * Functionname		: serializeStaticParameters
 * Input			: Vector of parameter values, StaticParameters object   
 * Output			: Returns the vector with the assigned values of struct object 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Assigns the struct values to the vector. The processing happens
 * 					  with the use of a structure 
 **************************************************************************************/
void serializeStaticParameters(const ModelOptimizer::StaticParameters &params, std::vector<double> &x)
{
	x.resize(12);
    x[0] = params.mass[0];
    x[1] = params.mass[1];
    x[2] = params.mass[2];
    x[3] = params.mass[3];
    x[4] = params.com_radius[0];
    x[5] = params.com_radius[1];
    x[6] = params.com_radius[2];
    x[7] = params.com_radius[3];
    x[8] = params.com_angle[0];
    x[9] = params.com_angle[1];
    x[10] = params.com_angle[2];
    x[11] = params.com_angle[3];
}

/****************************************************************************************
 * Functionname		: loadParams
 * Input			: string values to specify static or dynamic configurations  
 * Output			: -
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Robot's experiment data is stored locally for the optimization
 **************************************************************************************/
void ModelOptimizer::loadParams(string s)
{
	vector<double> inParamsDeser;
	double douPa = 0.0; 
	int countParam = 0;
	
	string packagePath;
	if(s[0] == 's')
	{
		inParamsDeser.resize(13);

		packagePath = ros::package::getPath("models_evaluation")+"/data/outparam_sta.dat";
	}
	else if(s[0] == 'm')
	{
		inParamsDeser.resize(17);
		packagePath = ros::package::getPath("models_evaluation")+"/data/outparam_dyn.dat";
	}
		
	const char *filePath_lp = packagePath.c_str();
	string strPa;
	ifstream ipParams(filePath_lp, ios::in);
	/// Check to ensure the file could be opened or not
	if(!ipParams)
	{
		cerr << "File could not be opened" << endl;
		exit(ONE);
	}
	while(ipParams >> strPa >> douPa)
	{
		inParamsDeser[countParam] = douPa;
		countParam++;
	}
	if(s[0] == 's')
	{	
		deserializeStaticParameters(inParamsDeser, param_data);
	}
	else if(s[0] == 'm')
	{
		deserializeDynamicParameters(inParamsDeser, param_data_dyn);		
	}
}

/****************************************************************************************
 * Functionname		: dynamicsErrorCalculation
 * Input			: Vector of parameter values, DynamicParameters object   
 * Output			: Returns the best parameters after optimization 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Objective function of the optimizer to calculate error
 **************************************************************************************/
double dynamicErrorCalculation(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
	deserializeDynamicParameters(x, param_data_dyn);
	modelOpti.setDynamicParameters(param_data_dyn);

	vector<double> torque_pos, torque_vel, torque_acc, torque_robot, torque_diff;
	torque_pos.resize(5);
	torque_vel.resize(5);
	torque_acc.resize(5);
	torque_robot.resize(5);
	torque_diff.resize(5);
	double error = 0;
	vector<double> retTrq;
	retTrq.resize(5);

 	for(int i = 0; i < poses; i++)
	{
		torque_pos[0] = data_new[i].pos[0];
		torque_pos[1] = data_new[i].pos[1];
		torque_pos[2] = data_new[i].pos[2];
		torque_pos[3] = data_new[i].pos[3];
		torque_pos[4] = data_new[i].pos[4];

		torque_vel[0] = data_new[i].vel[0];
		torque_vel[1] = data_new[i].vel[1];
		torque_vel[2] = data_new[i].vel[2];
		torque_vel[3] = data_new[i].vel[3];
		torque_vel[4] = data_new[i].vel[4];

		if(poses != (i+1))
		{
			torque_acc[0] = ((data_new[i+1].vel[0]-data_new[i].vel[0])/0.11);
			torque_acc[1] = ((data_new[i+1].vel[1]-data_new[i].vel[1])/0.11);
			torque_acc[2] = ((data_new[i+1].vel[2]-data_new[i].vel[2])/0.11);
			torque_acc[3] = ((data_new[i+1].vel[3]-data_new[i].vel[3])/0.11);
			torque_acc[4] = ((data_new[i+1].vel[4]-data_new[i].vel[4])/0.11);										
		}
			
		torque_robot[0] = data_new[i].trq[0];
		torque_robot[1] = data_new[i].trq[1];
		torque_robot[2] = data_new[i].trq[2];
		torque_robot[3] = data_new[i].trq[3];
		torque_robot[4] = data_new[i].trq[4];

		retTrq = modelOpti.getDynamicEffort(torque_pos,torque_vel,torque_acc);
			
		for(unsigned int diff=1;diff<5;diff++)
			torque_diff[diff] = retTrq[diff] - torque_robot[diff];
		
		for(uint j=1; j<5;j++)
		{
			error += torque_diff[j]*torque_diff[j]; 
		}
	}
	err_id << iteration_count_dyn << " "<< error << endl;
	if(error < HUGE_VAL)
    {
        min_error = error;
		best_params_dyn = param_data_dyn;			
	}
	if(iteration_count_dyn % 1000 == 0 && iteration_count_dyn > 0)
    {
        for(uint i=0; i<16; i++)
        {
            cout << " " << x[i] << endl;
        }
        cout << endl;
	}
	iteration_count_dyn++;

	return error;
}

/****************************************************************************************
 * Functionname		: errorCalculation
 * Input			: Vector of parameter values, StaticParameters object   
 * Output			: Returns the best parameters after optimization 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Objective function of the optimizer
 **************************************************************************************/
double errorCalculation(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
	deserializeStaticParameters(x, param_data);
	modelOpti.setStaticParameters(param_data);

	vector<double> torque_pos, torque_vel, torque_acc, torque_robot, torque_diff;
	torque_pos.resize(5);
	torque_vel.resize(5);
	torque_acc.resize(5);
	torque_robot.resize(5);
	torque_diff.resize(5);
	double error = 0;
	vector<double> retTrq;
	retTrq.resize(5);

 	for(int i = 0; i < poses; i++)
	{
		torque_pos[0] = data_new[i].pos[0];
		torque_pos[1] = data_new[i].pos[1];
		torque_pos[2] = data_new[i].pos[2];
		torque_pos[3] = data_new[i].pos[3];
		torque_pos[4] = data_new[i].pos[4];

		torque_robot[0] = data_new[i].trq[0];
		torque_robot[1] = data_new[i].trq[1];
		torque_robot[2] = data_new[i].trq[2];
		torque_robot[3] = data_new[i].trq[3];
		torque_robot[4] = data_new[i].trq[4];
		
		retTrq = modelOpti.getStaticEffort(torque_pos,torque_vel,torque_acc);

		for(unsigned int diff=1;diff<5;diff++)
			torque_diff[diff] = retTrq[diff] - torque_robot[diff];
		
		for(uint j=1; j<5;j++)
		{
			error += torque_diff[j]*torque_diff[j]; 
		}
	}
	err_id << iteration_count << " "<< error << endl;
	if(error < HUGE_VAL)
    {
        min_error = error;
		best_params = param_data;
		
	}
	if(iteration_count % 1000 == 0 && iteration_count > 0)
    {
        for(uint i=0; i<12; i++)
        {
            cout << " " << x[i] << endl;
        }
        cout << endl;
	}
	iteration_count++;

	return error;
}

/****************************************************************************************
 * Functionname		: getOptimizedParams
 * Input			: -
 * Output			: - 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Optimizer implementation with all the required functions for
 * 					  static joint effort calculations
 **************************************************************************************/
void ModelOptimizer::getOptimizedStaticParams()
{
	ModelOptimizer::StaticParameters init_params, params, params1;
	/// Variable declaration for nlopt
	nlopt::opt *optimizer = new nlopt::opt(nlopt::LN_COBYLA, NO_OF_PARAMS);
	
	params.mass[0]         = 0;
    params.mass[1]         = 0;
    params.mass[2]         = 0;
    params.mass[3]         = 0;
    params.com_radius[0]         = 0;
    params.com_radius[1]         = 0;
    params.com_radius[2]         = 0;
    params.com_radius[3]         = 0;
    params.com_angle[0]         = -M_PI/2;
    params.com_angle[1]         = -M_PI/2;
    params.com_angle[2]         = -M_PI/2;
    params.com_angle[3]         = -M_PI/2;
	
	vector<double> vLowerbounds;

	serializeStaticParameters(params, vLowerbounds);
	optimizer->set_lower_bounds(vLowerbounds);

	params1.mass[0]         = 3;
    params1.mass[1]         = 3;
    params1.mass[2]         = 3;
    params1.mass[3]         = 3;
    params1.com_radius[0]         = 0.155;
    params1.com_radius[1]         = 0.135;
    params1.com_radius[2]         = 0.230;
    params1.com_radius[3]         = 0.230;
    params1.com_angle[0]         = M_PI;
    params1.com_angle[1]         = M_PI;
    params1.com_angle[2]         = M_PI;
    params1.com_angle[3]         = M_PI;
    std::vector<double> vUpperbounds;
	serializeStaticParameters(params1, vUpperbounds);
	optimizer->set_upper_bounds(vUpperbounds);
	/// termination criterion
	optimizer->set_xtol_rel(1e-12);
	/// Optimizer function
	optimizer->set_min_objective(errorCalculation, &data_new);

	std::vector<double> x;
    init_params.mass[0]         = 0.6704054434195545;
    init_params.mass[1]         = 0.7065456362157415;
    init_params.mass[2]         = 0.9736744251972129;
    init_params.mass[3]         = 0.4127435232645671;
    init_params.com_radius[0]         = 0.06402917528867183;
    init_params.com_radius[1]         = 0.08795328939031184;
    init_params.com_radius[2]         = 0.04833327364289946;
    init_params.com_radius[3]         = 0.2189300979881188;
    init_params.com_angle[0]         = 0.2964549495469563;
    init_params.com_angle[1]         = 0.1307947824954063;
    init_params.com_angle[2]         = 0.002036069407585526;
    init_params.com_angle[3]         = -0.1850041679383009;

	serializeStaticParameters(init_params, x);
	/// To start the optimization
    double minf = 0.0;
	nlopt::result result = optimizer->optimize(x, minf);
	if(result < 0)
    {
        cout << "Optimization failed." << endl;
        return;
	}
	modelOpti.setStaticParameters(best_params);
	cout << "Iteration " << iteration_count << endl;
}

/****************************************************************************************
 * Functionname		: getOptimizedDynamicParams
 * Input			: -
 * Output			: - 
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Optimizer implementation with all the required functions for
 * 					  dynamic joint effort calculations
 **************************************************************************************/
void ModelOptimizer::getOptimizedDynamicParams()
{
	ModelOptimizer::DynamicParameters init_params, params, params1;
	/// Variable declaration for nlopt
	nlopt::opt *optimizer = new nlopt::opt(nlopt::LN_COBYLA, NO_OF_PARAMS_DYN);
	
	params.mass[0]         = 0.6;
    params.mass[1]         = 0.7;
    params.mass[2]         = 0.5;
    params.mass[3]         = 0.4;
    params.com_radius[0]         = 0;
    params.com_radius[1]         = 0;
    params.com_radius[2]         = 0;
    params.com_radius[3]         = 0;
    params.com_angle[0]         = -M_PI/2;
    params.com_angle[1]         = -M_PI/2;
    params.com_angle[2]         = -M_PI/2;
    params.com_angle[3]         = -M_PI/2;
    params.com_z[0]         = 0;
    params.com_z[1]         = 0;
    params.com_z[2]         = 0;
    params.com_z[3]         = 0;
    
	vector<double> vLowerbounds;

	serializeDynamicParameters(params, vLowerbounds);
	optimizer->set_lower_bounds(vLowerbounds);

	params1.mass[0]         = 1.6;
    params1.mass[1]         = 1.6;
    params1.mass[2]         = 1;
    params1.mass[3]         = 0.8;
    params1.com_radius[0]         = 0.155;
    params1.com_radius[1]         = 0.135;
    params1.com_radius[2]         = 0.230;
    params1.com_radius[3]         = 0.230;
    params1.com_angle[0]         = M_PI;
    params1.com_angle[1]         = M_PI;
    params1.com_angle[2]         = M_PI;
    params1.com_angle[3]         = M_PI;
    params1.com_z[0]         = 0.2;
    params1.com_z[1]         = 0.2;
    params1.com_z[2]         = 0.2;
    params1.com_z[3]         = 0.2;
    
    std::vector<double> vUpperbounds;
	serializeDynamicParameters(params1, vUpperbounds);
	optimizer->set_upper_bounds(vUpperbounds);
	/// termination criterion
	optimizer->set_xtol_rel(1e-12);
		/// Optimizer function
	optimizer->set_min_objective(dynamicErrorCalculation, &data_new);

	std::vector<double> x;
    init_params.mass[0]         = 0.6704054434195545;
    init_params.mass[1]         = 0.7065456362157415;
    init_params.mass[2]         = 0.9736744251972129;
    init_params.mass[3]         = 0.4127435232645671;
    init_params.com_radius[0]         = 0.06402917528867183;
    init_params.com_radius[1]         = 0.08795328939031184;
    init_params.com_radius[2]         = 0.04833327364289946;
    init_params.com_radius[3]         = 0.2189300979881188;
    init_params.com_angle[0]         = 0.2964549495469563;
    init_params.com_angle[1]         = 0.1307947824954063;
    init_params.com_angle[2]         = 0.002036069407585526;
    init_params.com_angle[3]         = -0.1850041679383009;
    init_params.com_z[0]         = 0.1;
    init_params.com_z[1]         = 0.12;
    init_params.com_z[2]         = 0.1;
    init_params.com_z[3]         = 0.1;
    
	serializeDynamicParameters(init_params, x);

	/// To start the optimization
    double minf = 0.0;
	nlopt::result result = optimizer->optimize(x, minf);
	if(result < 0)
    {
        cout << "Optimization failed." << endl;
        return;
	}
	modelOpti.setDynamicParameters(best_params_dyn);
	cout << "Iteration " << iteration_count_dyn << endl;
}

/****************************************************************************************
 * Functionname		: loadData
 * Input			: string with the mode of configurations
 * Output			: -
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: This function loads the trajectories velocities,
 * 					  and torque of the manipulator
 **************************************************************************************/
void ModelOptimizer::loadData(string s)
{	
	string packagePath;
	if(s[0] == 's')
	{
		data_new = new ModelOptimizer::State[41];
		packagePath = ros::package::getPath("models_evaluation")+"/data/inputs/Joint1/staticj1.dat";
		poses = 41;
		config[0] = 's';
	}
	else if(s[0] == 'm')
	{
		data_new = new ModelOptimizer::State[1503];
		packagePath = ros::package::getPath("models_evaluation")+"/data/inputs/Joint1/r_joint1.dat";
		poses = 1503;
		config[0] = 'm';
	}
	const char *filePath_ld = packagePath.c_str();
	unsigned int count_pos = 0, count_vel = 0, count_trq = 0;
	string str;
	ifstream inQ1read(filePath_ld, ios::in);
	/// Check to ensure the file could be opened or not
	if(!inQ1read)
	{
		cerr << "File could not be opened" << endl;
		exit(ONE);
	}

	double doublevalues[5];
	while(inQ1read >> str >> doublevalues[0] >> doublevalues[1] >> doublevalues[2] >>doublevalues[3] >>doublevalues[4])
	{		
		if(str == "Position")
		{
			data_new[count_pos].pos[0] = doublevalues[0];
			data_new[count_pos].pos[1] = doublevalues[1];
			data_new[count_pos].pos[2] = doublevalues[2];
			data_new[count_pos].pos[3] = doublevalues[3];
			data_new[count_pos].pos[4] = doublevalues[4];
			count_pos++;
		}
		
		if(str == "Velocity")
		{
			data_new[count_vel].vel[0] = doublevalues[0];
			data_new[count_vel].vel[1] = doublevalues[1];
			data_new[count_vel].vel[2] = doublevalues[2];
			data_new[count_vel].vel[3] = doublevalues[3];
			data_new[count_vel].vel[4] = doublevalues[4];
			count_vel++;
		}		
		
		if(str == "Torque")
		{
			data_new[count_trq].trq[0] = doublevalues[0];
			data_new[count_trq].trq[1] = doublevalues[1];
			data_new[count_trq].trq[2] = doublevalues[2];
			data_new[count_trq].trq[3] = doublevalues[3];
			data_new[count_trq].trq[4] = doublevalues[4];
			count_trq++;
		}
	}
	
	inQ1read.close();
}
