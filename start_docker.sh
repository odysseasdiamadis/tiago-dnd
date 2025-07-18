#!/bin/bash
#
set -x

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Variables for forwarding ssh agent into docker container
SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

# Settings required for having nvidia GPU acceleration inside the docker
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"

dpkg -l | grep nvidia-container-toolkit &> /dev/null
HAS_NVIDIA_TOOLKIT=$?
which nvidia-docker > /dev/null
HAS_NVIDIA_DOCKER=$?
if [ $HAS_NVIDIA_TOOLKIT -eq 0 ]; then
  docker_version=`docker version --format '{{.Client.Version}}' | cut -d. -f1`
  if [ $docker_version -ge 19 ]; then
	  DOCKER_COMMAND="docker run --gpus all"
  else
	  DOCKER_COMMAND="docker run --runtime=nvidia"
  fi
elif [ $HAS_NVIDIA_DOCKER -eq 0 ]; then
  DOCKER_COMMAND="nvidia-docker run"
else
  echo "Running without nvidia-docker, if you have an NVidia card you may need it"\
  "to have GPU acceleration"
  DOCKER_COMMAND="docker run"
fi

DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

PULSE_ARGS=""
if [[ $1 == "pulse" ]]; then
    PULSE_ARGS="-e PULSE_SERVER=unix:/run/user/1000/pulse/native -v /run/user/1000/pulse/native:/run/user/1000/pulse/native"
else
    PULSE_ARGS="-e PULSE_SERVER=127.0.0.1"
fi

xhost +

$DOCKER_COMMAND \
$DOCKER_USER_ARGS \
$DOCKER_GPU_ARGS \
$DOCKER_SSH_AUTH_ARGS \
$DOCKER_NETWORK_ARGS \
--privileged \
$PULSE_ARGS \
-e NVIDIA_VISIBLE_DEVICES=all \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-v ~/.config/pulse/cookie:/root/.config/pulse/cookie \
-v /dev/dsp:/dev/dsp \
-v "$HOME/exchange:/home/user/exchange" \
-v "$PWD/dnd_session.world:/tiago_public_ws/src/pal_gazebo_worlds/worlds/dnd.world" \
-v /var/run/docker.sock:/var/run/docker.sock \
-it -v /dev/snd:/dev/snd \
-v $(pwd)/src:/src registry.gitlab.com/brienza1/empower_docker:latest

