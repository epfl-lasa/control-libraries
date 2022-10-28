#!/usr/bin/env bash

BASE_TAG="latest"

BRANCH=$(git branch --show-current)

HELP_MESSAGE="Usage: build-test.sh [-b <branch>] [--base-tag <base-tag>] [-r] [-v]
Options:
  -b, --branch <branch>    Specify the branch of control libraries
                           that should be used to build the image.
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
    -b|--branch) BRANCH=$2; shift 2;;
    --base-tag) BASE_TAG=$2; shift 2;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift ;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

IMAGE_NAME=epfl-lasa/control-libraries/protocol/testing:"${BASE_TAG}"
BUILD_FLAGS+=(--build-arg BASE_TAG="${BASE_TAG}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}")

echo "Using control libraries branch ${BRANCH}"
BUILD_FLAGS+=(--build-arg BRANCH="${BRANCH}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies:"${BASE_TAG}" || exit 1
DOCKER_BUILDKIT=1 docker build . --file ./Dockerfile.protocol "${BUILD_FLAGS[@]}"
