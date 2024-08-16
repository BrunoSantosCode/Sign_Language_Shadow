#!/usr/bin/env python3

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


if __name__ == "__main__":
    # Init ROS
    rospy.init_node('get_shadow_joint_values')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Get Hand Pose
    hand_joints = hand_commander.get_joints_position()

    # Display
    print("\n", "Shadow Hand Joints:", "\n", hand_joints, "\n")

    exit()