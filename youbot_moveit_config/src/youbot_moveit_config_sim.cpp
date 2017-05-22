#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#define JNT_1 2.96244
#define JNT_2 1.04883
#define JNT_3 -2.43523
#define JNT_4 1.73184
#define JNT_5 2.91062

using namespace std;

float joint_pos_global[5], joint_vel_global[5], joint_eff_global[5]; 
unsigned int pos,vel,eff;

void callback(const sensor_msgs::JointState & msg)
{
	pos = msg.position.size();
	for(unsigned int jtstate = 0; jtstate < pos;jtstate++)
	{
		joint_pos_global[jtstate] = msg.position[jtstate];
	}
	vel = msg.velocity.size();
	for(unsigned int jtstate = 0; jtstate < vel;jtstate++)
	{
		joint_vel_global[jtstate] = msg.velocity[jtstate];
	}
	eff = msg.effort.size();
	for(unsigned int jtstate = 0; jtstate < eff;jtstate++)
	{
		joint_eff_global[jtstate] = msg.effort[jtstate];
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_moveit_config",ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(1);
	std::ofstream outFile;
	ros::NodeHandle nh; 
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	outFile.open("/home/jp/toyoubot_joint4.dat", ios::out);
	spinner.start();
	move_group_interface::MoveGroup group("youbotmanipulator");
	ros::Subscriber sub; 
	sub = nh.subscribe ("/joint_states", 10, callback); 

	/// group.setRandomTarget();
	/// group.move();
	move_group_interface::MoveGroup::Plan plan;
	/** std::map<std::string, double> joints;
	joints["arm_joint_1"] =  2;
	joints["arm_joint_2"] = 2.67;
	joints["arm_joint_3"] = 0;
	joints["arm_joint_4"] =  0;
	joints["arm_joint_5"] = 2;
	group.setJointValueTarget(joints); ***/

	std::map<std::string, double> joints;
	joints["arm_joint_1"] = 2.96244;
	joints["arm_joint_2"] = 1.04883;
	joints["arm_joint_3"] = -2.43523;
	joints["arm_joint_4"] =  1.73184;
	joints["arm_joint_5"] = 2.91062;
	group.setJointValueTarget(joints);
	group.move();
	//bool success;
	//success = group.plan(plan);
	
	for(unsigned int i= 0; i<50; i++)
	{
		std::vector<double> joints = group.getRandomJointValues();
		joints[0] = JNT_1;
		joints[1] = JNT_2;
		joints[2] = JNT_3;
		joints[3] = JNT_4;
		group.setJointValueTarget(joints);
		
		if (!group.plan(plan))
		{
			ROS_INFO("Unable to create motion plan.  Aborting.");
			continue;
		}
		else
		{
			std::cout << "loop: " << i << std::endl;
			group.execute(plan);
			/// std::vector<double> joints1 (group.getCurrentJointValues());
			for (unsigned j = 0; j < joints.size(); j++)
			{
				outFile << joints[j] << " ";
				std::cout << joints[j] << " ";
			}
			
			outFile << std::endl;
			std::cout << std::endl;
			group.stop();
			/*ros::Rate rate(20); //Hz 
			joint_state.position = joints;

			for(unsigned int jtstate = 0; jtstate < pos;jtstate++)
				cout << " " << joint_state.position[jtstate];
				cout << " " << joint_pos_global[jtstate];
			
			cout << endl;
			*/
		}
	}
	outFile.close();
	ros::shutdown();
}
