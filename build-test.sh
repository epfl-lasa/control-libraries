#!/usr/bin/env bash
# Build a docker image to compile the library and run tests

MULTISTAGE_TARGET="build-testing"
BUILD_TESTING="ON"
BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

NAME=$(echo "${PWD##*/}" | tr _ -)/$MULTISTAGE_TARGET
TAG="latest"

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(--build-arg "BUILD_TESTING=${BUILD_TESTING}")
BUILD_FLAGS+=(--build-arg "BUILD_CONTROLLERS=${BUILD_CONTROLLERS}")
BUILD_FLAGS+=(--build-arg "BUILD_DYNAMICAL_SYSTEMS=${BUILD_DYNAMICAL_SYSTEMS}")
BUILD_FLAGS+=(--build-arg "BUILD_ROBOT_MODEL=${BUILD_ROBOT_MODEL}")
BUILD_FLAGS+=(-t "${NAME}:${TAG}")

if [ "$REBUILD" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}"  .
