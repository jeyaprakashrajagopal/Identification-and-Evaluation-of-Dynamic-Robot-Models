/****************************************************************************************
 * Filename 	: youbot_moveit_config.cpp
 * Path			: youbot_moveit_config/src
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: source file for conducting robot experiments using moveit
 **************************************************************************************/
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <youbot_driver/testing/YouBotArmTest.hpp>
#define JNT_1 2.96244
#define JNT_2 1.04883
#define JNT_3 -2.43523
#define JNT_4 1.73184
#define JNT_5 2.91062
#define JT1_LINENO 42
#define JT2_LINENO 44
#define JT3_LINENO 43
#define JT4_LINENO 51
#define JT5_LINENO 51
using namespace std;
using namespace youbot;
float joint_pos_global[5], joint_vel_global[5], joint_eff_global[5]; 
unsigned int pos,vel,eff;
std::ofstream outFile("/home/jp/r_joint1.dat", ios::out);

/****************************************************************************************
 * Functionname		: callback
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: To record the data from the joint_state message from ros interface
 **************************************************************************************/
void callback(const sensor_msgs::JointState & msg)
{
	outFile << "Position";
	pos = msg.position.size()-2;
	for(unsigned int jtstate = 0; jtstate < pos;jtstate++)
	{
		joint_pos_global[jtstate] = msg.position[jtstate];
		outFile << " " << joint_pos_global[jtstate];
	}
	outFile << endl;

	outFile << "Velocity";
	vel = msg.velocity.size()-2;
	for(unsigned int jtstate = 0; jtstate < vel;jtstate++)
	{
		joint_vel_global[jtstate] = msg.velocity[jtstate];
		outFile << " " << joint_vel_global[jtstate];
	}
	outFile << endl;

	outFile << "Torque";	
	eff = msg.effort.size()-2;
	for(unsigned int jtstate = 0; jtstate < eff;jtstate++)
	{
		joint_eff_global[jtstate] = msg.effort[jtstate];
		outFile << " " << joint_eff_global[jtstate];
	}
	outFile << endl;
}

/****************************************************************************************
 * Functionname		: main
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Main function for the moveit to record the data
 **************************************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_moveit_config",ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(1);
	string inPath  = ros::package::getPath("youbot_moveit_config");
	string joint_angles = inPath + "/data/toyoubot_joint1.dat";
	const char *filePath = joint_angles.c_str();
	int counter = 0;
	ifstream inFile(filePath,ios::in);
	ofstream outFile_static;
	outFile_static.open("/home/jp/staticj1.dat",ios::out);
	/// Check to ensure the file could be opened or not
	if(!inFile)
	{
		cerr << "File could not be opened" << endl;
		exit(1);
	}
	/// Handle to connect to the master to publish or subscribe
	ros::NodeHandle nh; 
	/// Planning interface declarations
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	spinner.start();
	/// initializing the interface to the group that has been created 
	move_group_interface::MoveGroup group("youbotmanipulator");
	/// To subscribe to the joint_state 
	ros::Subscriber sub; 
	/// Planner initialization
	move_group_interface::MoveGroup::Plan plan;

	/// Initial configurations
	std::vector<double> joints;
	youbot::JointSensedTorque torque;
	joints.resize(5);
	joints[0] = 2.96244;
	joints[1] = 1.04883;
	joints[2] = -2.43523;
	joints[3] =  1.73184;
	joints[4] = 2.91062;
	/// Set the joint angles to the group
	group.setJointValueTarget(joints);
	std::vector<double> joints2 (group.getCurrentJointValues());
	cout << joints2[0] << " " << joints2[1] << " " << joints2[2] << " " << joints2[3] << " " << joints2[4] << endl;
	/// Validating the plan for collision
	if(!group.plan(plan))
	{ 
		cout << "plan failed" << endl;
		exit(1);
	}
	else
	{
		/// Executing a plan in the group
		group.execute(plan);
	}
	/// to subscribe to the joint_states message
	sub = nh.subscribe ("/joint_states_throttle", 1, callback); 

	while(!inFile.eof())
	{
		std::vector<double> joints;
		joints.resize(5);
		
		///read out a line of the file
		inFile >> joints[0] >> joints[1] >> joints[2] >> joints[3] >> joints[4];
		cout << "Position "<< joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << endl;
		/// To check the eof 
		counter++;
		if(counter == JT1_LINENO)
		{
			break;
		}
		/// Set the joint angles to the group
		group.setJointValueTarget(joints);
		/// Validating the plan for collision
		if (!group.plan(plan))
		{
			ROS_INFO("Unable to create motion plan.  Aborting.");
			continue;
		}
		else
		{
			/// To execute the desired trajectories
			group.execute(plan);
			group.stop();
		}
	}
	/// Files pointers are closed
	outFile.close();
	outFile_static.close();
	ros::shutdown();
}
