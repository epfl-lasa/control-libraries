#!/usr/bin/env bash

BASE_TAG="latest"

BUILD_CONTROLLERS="ON"
BUILD_DYNAMICAL_SYSTEMS="ON"
BUILD_ROBOT_MODEL="ON"

HELP_MESSAGE="Usage: build-test.sh [--base-tag <base-tag>] [-r] [-v]
Options:
  --base-tag <base-tag>    Tag of the development image.
                           (default: ${BASE_TAG})
  -r, --rebuild            Rebuild the image using the docker
                           --no-cache option.
  -v, --verbose            Use the verbose option during the building
                           process.
  -h, --help               Show this help message.
"

BUILD_FLAGS=()
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    --base-tag) BASE_TAG=$2; shift 2;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift ;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

IMAGE_NAME=epfl-lasa/control-libraries/source/testing:"${BASE_TAG}"

BUILD_FLAGS+=(--build-arg "BASE_TAG=${BASE_TAG}")
BUILD_FLAGS+=(--build-arg "BUILD_CONTROLLERS=${BUILD_CONTROLLERS}")
BUILD_FLAGS+=(--build-arg "BUILD_DYNAMICAL_SYSTEMS=${BUILD_DYNAMICAL_SYSTEMS}")
BUILD_FLAGS+=(--build-arg "BUILD_ROBOT_MODEL=${BUILD_ROBOT_MODEL}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies:"${BASE_TAG}" || exit 1
DOCKER_BUILDKIT=1 docker build . --file ./Dockerfile.source "${BUILD_FLAGS[@]}"
