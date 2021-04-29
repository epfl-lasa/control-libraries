#!/usr/bin/env bash
# Build a docker image to compile the library and run tests
MULTISTAGE_TARGET="runtime-demonstrations"

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
TARGET_SCRIPT=${1}

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(-t "${NAME}:${TAG}")

if [ "$REBUILD" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

MULTISTAGE_SOURCE_TARGET="source-dependencies"
DOCKER_BUILDKIT=1 docker build --target "${MULTISTAGE_SOURCE_TARGET}" -t "control-libraries/${MULTISTAGE_SOURCE_TARGET}" ../.. || exit
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit

if [ -z "${1}" ]; then
  docker run -it --rm "${NAME}:${TAG}"
else
  docker run --rm "${NAME}:${TAG}" "./${1}"
fi
