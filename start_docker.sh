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
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Display settings
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"

# Just use basic docker run
DOCKER_COMMAND="docker run"

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
-v $ACTUAL_HOME/.config/pulse/cookie:/home/user/.config/pulse/cookie \
-v /dev/snd:/dev/snd \
-v "$ACTUAL_HOME/exchange:/home/user/exchange" \
-v "$PWD/dnd_session.world:/tiago_public_ws/src/pal_gazebo_worlds/worlds/dnd.world" \
-v /var/run/docker.sock:/var/run/docker.sock \
-v $(pwd)/src:/src \
-it \
registry.gitlab.com/brienza1/empower_docker:latest