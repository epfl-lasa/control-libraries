#!/usr/bin/env bash
# Build a docker image to compile the library and run the demo

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

NAME=$(echo "${PWD##*/}" | tr _ -)
TAG="latest"

UID="$(id -u "${USER}")"
GID="$(id -g "${USER}")"

BUILD_FLAGS=(-t "${NAME}:${TAG}")

if [ "$REBUILD" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .