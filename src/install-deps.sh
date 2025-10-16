#!/bin/bash

# This script installs the required dependancies and libs required in this project.
# Run it after starting this docker and after calling ./join.sh.

# REMEMBER: you should launch this script from root
if [ "$PWD" != "/" ]; then
    echo "We are NOT in the root directory. Moving into: /"
    cd /
fi

# update system (fast-try without version update)
# sudo apt update --yes
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
source "$VENV_DIR/bin/activate"


# IMPORTANT: to make this script work you should run:
# source ./src/install-deps.sh
# This is needed to make the venv activation persistant. In case of updates to venv env, remove src/venv dir and re-run this script.