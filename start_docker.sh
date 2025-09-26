#!/bin/bash
#
set -x

# Get the actual user info when running with sudo
ACTUAL_USER=${SUDO_USER:-$USER}
ACTUAL_HOME=$(getent passwd $ACTUAL_USER | cut -d: -f6)
ACTUAL_UID=$(id -u $ACTUAL_USER)

# Basic user environment
export LOCAL_USER_ID=$ACTUAL_UID
export LOCAL_GROUP_ID=$(id -g $ACTUAL_USER)
export LOCAL_GROUP_NAME=$(id -gn $ACTUAL_USER)

# Just use basic docker run
DOCKER_COMMAND="docker run"

DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Display settings
DOCKER_GPU_ARGS="--gpus all --env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
DOCKER_GPU_ARGS="--gpus all \
                 --env DISPLAY \
                 --env QT_X11_NO_MITSHM=1 \
                 --env LIBGL_ALWAYS_INDIRECT=0 \
                 --env LIBGL_ALWAYS_SOFTWARE=0 \
                 --env MESA_GL_VERSION_OVERRIDE=3.3 \
                 --env __GL_SYNC_TO_VBLANK=0 \
                 --env XAUTHORITY=$ACTUAL_HOME/.Xauthority \
                 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
                 --volume=$ACTUAL_HOME/.Xauthority:/home/user/.Xauthority:rw \
                 --device /dev/dri:/dev/dri"


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


# Network
DOCKER_NETWORK_ARGS="--net host"

# Audio - use PulseAudio socket from actual user
PULSE_ARGS="-e PULSE_SERVER=unix:/run/user/$ACTUAL_UID/pulse/native \
            -v /run/user/$ACTUAL_UID/pulse:/run/user/$ACTUAL_UID/pulse"

# Add audio group
AUDIO_GROUP_ID=$(getent group audio | cut -d: -f3)

xhost +
# -u $LOCAL_USER_ID \

$DOCKER_COMMAND \
$DOCKER_USER_ARGS \
$DOCKER_GPU_ARGS \
$DOCKER_NETWORK_ARGS \
--group-add $AUDIO_GROUP_ID \
--privileged \
$PULSE_ARGS \
-e NVIDIA_VISIBLE_DEVICES=all \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-v $ACTUAL_HOME/.config/pulse/cookie:/home/user/.config/pulse/cookie \
-v /dev/snd:/dev/snd \
-v "$ACTUAL_HOME/exchange:/home/user/exchange" \
-v "$PWD/dnd_1players.world:/tiago_public_ws/src/pal_gazebo_worlds/worlds/dnd_1players.world" \
-v "$PWD/dnd_2players.world:/tiago_public_ws/src/pal_gazebo_worlds/worlds/dnd_2players.world" \
-v "$PWD/dnd_2players_error.world:/tiago_public_ws/src/pal_gazebo_worlds/worlds/dnd_2players_error.world" \
-v "$PWD/dnd_3players.world:/tiago_public_ws/src/pal_gazebo_worlds/worlds/dnd_3players.world" \
-v /var/run/docker.sock:/var/run/docker.sock \
-v $(pwd)/src:/src \
-it \
registry.gitlab.com/brienza1/empower_docker:latest
