#!/bin/bash
MULTISTAGE_TARGET="development"

NAME=$(echo "${PWD##*/}" | tr _ -)/"${MULTISTAGE_TARGET}"
TAG=$(echo "$1" | tr _/ -)

ISISOLATED=true # change to  false to use host network

NETWORK=host
if [ "${ISISOLATED}" = true ]; then
    docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
    NETWORK=isolated
fi

if [ -z "$TAG" ]; then
    TAG="latest"
fi

VOLUME_NAME=$(echo "${NAME}_lib_vol" | tr / -)

#create a shared volume to store the lib folder
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/source/" \
    --opt o="bind" \
    "${VOLUME_NAME}"

xhost +
docker run \
    --privileged \
    --net="${NETWORK}" \
    -it \
    --rm \
    --volume="${VOLUME_NAME}:/home/udev/control_lib/:rw" \
    "${NAME}:${TAG}"
