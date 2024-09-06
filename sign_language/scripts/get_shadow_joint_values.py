#!/usr/bin/env python3

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

def format_joint_positions(joint_positions):
    # Round the joint positions and format as YAML
    formatted_positions = {}
    for joint, position in joint_positions.items():
        formatted_positions[joint] = round(position, 4)
    
    # Convert to a formatted string like YAML
    yaml_output = "\n".join([f"{joint}: {formatted_positions[joint]: 0.4f}" for joint in formatted_positions])
    return yaml_output

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('get_shadow_joint_values')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Get Hand Pose
    hand_joints = hand_commander.get_joints_position()

     # Format and print the joint positions in YAML format
    yaml_formatted_joints = format_joint_positions(hand_joints)
    print("\n", "Shadow Hand Joints (YAML format):", "\n", yaml_formatted_joints, "\n")

    exit()