# Sign Language Shadow

## 📖 Overview
This repository contains a special implementation for translating Portuguese text into Portuguese Sign Language (LGP) using the **Shadow Robotic Hand**. The project aims to simulate sign language gestures based on input sentences, utilizing predefined joint configurations for each character in the Portuguese alphabet. The robotic hand reproduces these gestures to represent the corresponding letters.

## ✨ Features
- **Text-to-Sign Translation**: Translate a user-provided Portuguese sentence into corresponding robotic hand gestures.
- **Alphabet Support**: The YAML configuration file contains joint configurations for each letter in the Portuguese alphabet.
- **Real-Time Feedback**: Letters are displayed in the terminal with real-time robotic hand movement for each corresponding character.
- **Error Handling**: Characters not present in the dictionary are marked in red, while valid characters are highlighted in green.

## 🛠️ Scripts Functionality

### [`sign_language_shadow.py`](sign_language/src/sign_language_shadow.py)
  - Main Python script
  - The script reads a sentence input by the user, checks each character, and if available, moves the robotic hand to the corresponding joint positions.
  - If a space character is encountered, the script pauses for 2 seconds to indicate a word break.
  - Valid characters are highlighted in green and move the robotic hand.
  - Invalid or unsupported characters are highlighted in red and skipped.

### [`get_shadow_joint_values.py`](sign_language/scripts/get_shadow_joint_values.py)
  - Acquires from Shadow Hand the current joint positions and ouput the values in YAML format.

### [`sign_language_pt_alphabet.yaml`](sign_language/config/sign_language_pt_alphabet.yaml)
  - Portuguese Sign Language dictionary in YAML format
  - Contains the joint values for each letter

## 📦 Dependencies

This project relies on several key dependencies:

  - ROS (Robot Operating System): Manages communication with the robotic hand.
  - sr_robot_commander: Shadow software for controlling the Shadow Hand.
  - YAML: Used to load joint positions for sign language gestures.
  - termcolor: Provides colored output for terminal feedback.
