#!/bin/bash

# This script installs the required dependancies and libs required in this project.
# Run it after starting this docker and after calling ./join.sh.

# REMEMBER: you should launch this script from root
if [ "$PWD" != "/" ]; then
    echo "We are NOT in the root directory. Moving into: /"
    cd /
fi

# update system
sudo apt update --yes
sudo apt install python3-venv --yes

# libs for audio I/O
sudo apt install mpg123 --yes
sudo apt-get install portaudio19-dev --yes
sudo apt install -y alsa-utils
sudo apt-get install -y libasound2-plugins


# Define the virtual environment directory
VENV_DIR="src/venv/"
REQUIREMENTS_FILE="src/config/tiago_requirements.txt"

# Check if the directory exists
if [ -d "$VENV_DIR" ]; then
    echo "Virtual environment found at ./$VENV_DIR"
else
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"
    pip install -r $REQUIREMENTS_FILE
    echo "Virtual environment created at ./$VENV_DIR"
fi

# Activate the virtual environment - not useful for now
# source "$VENV_DIR/bin/activate"


# libs needed to use the models required for tiago brain
# pip install transformers gtts flask 
# pip install -U openai-whisper
# pip install pyaudio


# # pip install opencv-python tensorflow face_recognition numpy scipy


# # face detection
# #pip install face-recognition
# pip install tensorflow==2.11.0 deepface
# pip install ultralytics opencv-python
# pip install supervision pillow
# pip install deepface



# remember to: (try fixing drivers)
# glxinfo | grep -i 'OpenGL renderer'
# sudo add-apt-repository ppa:kisak/kisak-mesa
# sudo apt update --fix-missing
# sudo apt upgrade --fix-missing
# sudo apt install mesa-utils --yes