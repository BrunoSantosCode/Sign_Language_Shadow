#!/usr/bin/env python3

import yaml
import rospkg
from termcolor import colored

# Define Joint Limits
JOINT_LIMITS = {
    "rh_FFJ1": {"min":  0.000, "max": 1.571},
    "rh_FFJ2": {"min":  0.000, "max": 1.571},
    "rh_FFJ3": {"min": -0.262, "max": 1.571},
    "rh_FFJ4": {"min": -0.349, "max": 0.349},
    "rh_MFJ1": {"min":  0.000, "max": 1.571},
    "rh_MFJ2": {"min":  0.000, "max": 1.571},
    "rh_MFJ3": {"min": -0.262, "max": 1.571},
    "rh_MFJ4": {"min": -0.349, "max": 0.349},
    "rh_RFJ1": {"min":  0.000, "max": 1.571},
    "rh_RFJ2": {"min":  0.000, "max": 1.571},
    "rh_RFJ3": {"min": -0.262, "max": 1.571},
    "rh_RFJ4": {"min": -0.349, "max": 0.349},
    "rh_LFJ1": {"min":  0.000, "max": 1.571},
    "rh_LFJ2": {"min":  0.000, "max": 1.571},
    "rh_LFJ3": {"min": -0.262, "max": 1.571},
    "rh_LFJ4": {"min": -0.349, "max": 0.349},
    "rh_LFJ5": {"min":  0.000, "max": 0.785},
    "rh_THJ1": {"min": -0.261, "max": 1.571},
    "rh_THJ2": {"min": -0.524, "max": 0.524},
    "rh_THJ3": {"min": -0.209, "max": 0.209},
    "rh_THJ4": {"min":  0.000, "max": 1.221},
    "rh_THJ5": {"min": -1.047, "max": 1.047},
    "rh_WRJ1": {"min": -0.698, "max": 0.488},
    "rh_WRJ2": {"min": -0.489, "max": 0.140}
}

# Check if joint value is within limits
def check_joint_limits(character_name, joints):
    for joint, value in joints.items():
        if joint in JOINT_LIMITS:
            limits = JOINT_LIMITS[joint]
            if not (limits['min'] <= value <= limits['max']):
                # Print joints that are out of limits
                print(f"{character_name} - {colored(joint, 'red')}: {colored(value, 'red')} "f"(Limit: {limits['min']} to {limits['max']})")


if __name__ == "__main__":

    # Load sign language alphabet
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sign_language')
    yaml_file = package_path + '/config/sign_language_pt_alphabet.yaml'
    with open(yaml_file, 'r') as file:
        yaml_data = yaml.safe_load(file)
    
    # Check joint limits
    for character_name, joints in yaml_data.items():
        check_joint_limits(character_name, joints)

    print(colored("Done!", "green"))