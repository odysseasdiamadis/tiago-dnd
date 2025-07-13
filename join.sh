#!/bin/bash

CONT_ID=$(docker ps | grep brienza1 | awk '{ print $1 }' | head -1)

if [[ -z $CONT_ID ]]; then
    echo "Container not found"
fi

#docker exec -it $CONT_ID bash

# start up environment with libs
docker exec -it "$CONT_ID" bash -c "/src/install-deps.sh"