#!/usr/bin/env bash

MULTISTAGE_TARGET="testing"
IMAGE_NAME=epfl-lasa/control-libraries/source/"${MULTISTAGE_TARGET}"
IMAGE_TAG="latest"

BUILD_TESTING="ON"
BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"

HELP_MESSAGE="Usage: build-test.sh [-r] [-v]
Options:
  -r, --rebuild                   Rebuild the image using the docker
                                  --no-cache option
  -v, --verbose                   Use the verbose option during the building
                                  process
"
BUILD_FLAGS=()

while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -r|--rebuild) BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(--build-arg "BUILD_TESTING=${BUILD_TESTING}")
BUILD_FLAGS+=(--build-arg "BUILD_CONTROLLERS=${BUILD_CONTROLLERS}")
BUILD_FLAGS+=(--build-arg "BUILD_DYNAMICAL_SYSTEMS=${BUILD_DYNAMICAL_SYSTEMS}")
BUILD_FLAGS+=(--build-arg "BUILD_ROBOT_MODEL=${BUILD_ROBOT_MODEL}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies
DOCKER_BUILDKIT=1 docker build . --file ./Dockerfile.source "${BUILD_FLAGS[@]}"
