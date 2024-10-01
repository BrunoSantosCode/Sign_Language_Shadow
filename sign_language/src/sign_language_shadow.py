#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import unicodedata
from termcolor import colored
from sr_robot_commander.sr_hand_commander import SrHandCommander

previous_chr = ''

def remove_accent(input_str):
    return ''.join(
        (char for char in unicodedata.normalize('NFD', input_str)
         if unicodedata.category(char) != 'Mn')
    )

def special_cases(prev_chr, chr):
    if prev_chr == chr:
        return True
    if (chr=='M') and ((prev_chr!='N') and (prev_chr!='S')):
        return True
    if (chr=='N') and ((prev_chr!='M') and (prev_chr!='S')):
        return True
    if (chr=='S') and ((prev_chr!='M') and (prev_chr!='N')):
        return True
    if chr=='E' and prev_chr=='M':
        return True
    return False
    
def translate_to_sign_language(text, hand_commander, sign_dict):
    global previous_chr
    for raw_char in text:
        char = remove_accent(raw_char)
        if char == ' ':
            print(" ", end='', flush=True)
            rospy.sleep(1.0)
        elif char.upper() in sign_dict:
            if special_cases(previous_chr, char.upper()):
                hand_commander.move_to_joint_value_target_unsafe(
                    joint_states=sign["default"],
                    time=0.25, wait=True, angle_degrees=False
                )
            print(colored(raw_char, 'green'), end='', flush=True)
            hand_commander.move_to_joint_value_target_unsafe(
                joint_states=sign_dict[char.upper()],
                time=1.0, wait=True, angle_degrees=False
            )
            previous_chr = char.upper()
            rospy.sleep(0.25)
        else:
            print(colored(char, 'red'), end='', flush=True)
    print()
        

if __name__ == "__main__":
    # Init ROS
    rospy.init_node('sign_language_shadow')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    speed = 0.85
    hand_commander.set_max_velocity_scaling_factor(speed)
    hand_commander.set_max_acceleration_scaling_factor(speed)

    # Load sign language alphabet
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sign_language')
    yaml_file = package_path + '/config/sign_language_pt_alphabet.yaml'
    with open(yaml_file, 'r') as file:
        sign = yaml.safe_load(file)

    # Display available characters in the dictionary
    available_characters = ', '.join([char for char in sign.keys() if char != 'default'])
    print("\nAvailable characters in the sign language dictionary:")
    print(colored(available_characters, 'green'))

    # Set Shadow Hand default position
    hand_commander.move_to_joint_value_target_unsafe(
        joint_states=sign['default'],
        time=1.0, wait=True, angle_degrees=False
    )

    while True:
        # Ask the user for the input
        print()
        text = input("Please enter a sentence to translate into sign language (or type 'exit' to quit): ")

        if text.lower() == 'exit':
            print("\nExiting...")
            break

        # Translate text to sign language
        translate_to_sign_language(text, hand_commander, sign)

    # Return Shadow Hand to default position
    hand_commander.move_to_joint_value_target_unsafe(
        joint_states=sign["default"],
        time=1.0, wait=True, angle_degrees=False
    )

    exit()