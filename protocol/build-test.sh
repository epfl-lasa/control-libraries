#!/usr/bin/env bash

MULTISTAGE_TARGET="testing"
IMAGE_NAME=epfl-lasa/control-libraries/clproto
IMAGE_TAG="${MULTISTAGE_TARGET}"
BRANCH=$(git branch --show-current)

BUILD_TESTING="ON"

HELP_MESSAGE="Usage: build-test.sh [-b <branch>] [-r] [-v]
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
    -h|--help) echo "${HELP_MESSAGE}"; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

echo "Using control libraries branch ${BRANCH}"
BUILD_FLAGS+=(--build-arg BRANCH="${BRANCH}")
BUILD_FLAGS+=(--build-arg BUILD_TESTING="${BUILD_TESTING}")
BUILD_FLAGS+=(--target "${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies || exit 1
DOCKER_BUILDKIT=1 docker build . --file ./Dockerfile.protocol "${BUILD_FLAGS[@]}"
