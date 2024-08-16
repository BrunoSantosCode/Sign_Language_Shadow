#!/usr/bin/env python3

import yaml
import rospy
import rospkg
from termcolor import colored
from sr_robot_commander.sr_hand_commander import SrHandCommander

def translate_to_sign_language(text, hand_commander, sign_dict):
    for char in text:
        if char == ' ':
            print(" ", end='', flush=True)
            rospy.sleep(2.0)
        elif char.upper() in sign_dict:
            print(colored(char, 'green'), end='', flush=True)
            hand_commander.move_to_joint_value_target_unsafe(
                joint_states=sign_dict[char.upper()],
                time=1.0, wait=True, angle_degrees=False
            )
        else:
            print(colored(char, 'red'), end='', flush=True)
    print()
        

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('sign_language_shadow')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    speed = 1.0
    hand_commander.set_max_velocity_scaling_factor(speed)
    hand_commander.set_max_acceleration_scaling_factor(speed)

    # Load sign language alphabet
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sign_language')
    yaml_file = package_path + '/config/sign_language_pt_alphabet.yaml'
    with open(yaml_file, 'r') as file:
        sign = yaml.safe_load(file)

    # Display available characters in the dictionary
    available_characters = ', '.join(sign.keys())
    print("Available characters in the sign language dictionary:")
    print(colored(available_characters, 'green'))

    # Ask the user for the input
    text = input("Please enter a phrase to translate to sign language: ")

    # Translate text to sign language
    translate_to_sign_language(text, hand_commander, sign)

    exit()