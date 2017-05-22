#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

rospy.init_node("movegroup")
print "============ Starting tutorial setup"
robot = moveit_commander.RobotCommander()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print "============"
group = moveit_commander.MoveGroupCommander("youbotmanipulator")

#group.set_named_target('candle')
#group.go()
#group.set_named_target('candle')
#group.set_start_state()
#~ group_variable_values = group.get_current_joint_values()
#~ print "============ Joint values: ", group_variable_values
#~ group_variable_values[0] = 2.96244
#~ group_variable_values[1] = 2.04883
#~ group_variable_values[2] = -0.43523
#~ group_variable_values[3] = 1.73184
#~ group_variable_values[4] = 2.91062
#~ group.set_joint_value_target(group_variable_values)
#~ #plan2 = group.plan()
#~ print "plan result ", group.plan()
#~ group.go()
#print " random values: ", group.get_random_joint_values()
group.get_active_joints()
group_variable_values1 = group.get_random_joint_values()
group_variable_values1[0] = 2.96244
group.set_joint_value_target(group_variable_values1)
print "============ Joint values: ", group_variable_values1
plan2= group.plan()
group.go()
#group.set_start_state_to_current_state()
 
#print "============ Candle Joint values: ", group_variable_values

print "============ Waiting while RVIZ displays plan2..."
rospy.sleep(5)

print "============ STOPPING"
