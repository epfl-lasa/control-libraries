#!/usr/bin/env bash

IMAGE_NAME=epfl-lasa/control-libraries/control-loop-examples
IMAGE_TAG=latest

BRANCH=$(git branch --show-current)

HELP_MESSAGE="Usage: run-demo.sh [-b <branch>] [-r] [-v]
Options:
  -b, --branch <branch>    Specify the branch of control libraries
                           that should be used to build the image.
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
    -b|--branch) BRANCH=$2; shift 2;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

echo "Using control libraries branch ${BRANCH}"
BUILD_FLAGS+=(--build-arg CONTROL_LIBRARIES_BRANCH="${BRANCH}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . -t "${IMAGE_NAME}:${IMAGE_TAG}" || exit 1

docker run -it --rm "${IMAGE_NAME}:${IMAGE_TAG}"