#!/bin/sh

# This script installs the required dependancies and libs required in this project.
# Run it after starting this docker and after calling ./join.sh.

sudo apt update --yes

# libs for audio I/O
sudo apt install mpg123 --yes
sudo apt-get install portaudio19-dev --yes
sudo apt install -y alsa-utils
sudo apt-get install -y libasound2-plugins

# libs needed to use the models required for tiago brain
pip install transformers gtts flask 
pip install -U openai-whisper
pip install pyaudio


# pip install opencv-python tensorflow face_recognition numpy scipy


# face detection
pip install tensorflow==2.11.0 deepface
pip install ultralytics opencv-python
pip install supervision pillow
pip install deepface
