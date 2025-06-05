#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import os
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np


def twist_callback(msg):

    angular_vel_x = msg.angular.x
    angular_vel_y = msg.angular.y
    angular_vel_z = msg.angular.z
    linear_vel_x = msg.linear.x
    linear_vel_y = msg.linear.y
    linear_vel_z = msg.linear.z

    """ gripper_pos = 0.0
    jnt_speed = 0.2
    arm.MoveJ(np.array([angular_vel_x, angular_vel_y, angular_vel_z, linear_vel_x, linear_vel_y, linear_vel_z]), gripper_pos, jnt_speed)
 """

    """ gripper_pos = 0.0
    cartesian_speed = 0.1
    arm.MoveL(np.array([angular_vel_x, angular_vel_y, angular_vel_z, linear_vel_x, linear_vel_y, linear_vel_z]), gripper_pos, cartesian_speed) """
    arm.startTrack(armState.CARTESIAN)
    arm.cartesianCtrlCmd(np.array([linear_vel_x, linear_vel_y, linear_vel_z, angular_vel_x, angular_vel_y, angular_vel_z, 0]), 0.2, 0.2)

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node('robot_arm_controller')
    np.set_printoptions(precision=3, suppress=True)
    
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    armState = unitree_arm_interface.ArmFSMState
    arm.loopOn()

    #arm.labelRun("forward")

    rospy.Subscriber('/spacenav/twist', Twist, twist_callback)

    rospy.spin()

    #arm.loopOff()

