#!/bin/bash
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

if [ "$REBUILD" -eq 1 ]; then
    docker build \
        --no-cache \
        --build-arg UID="${UID}" \
        --build-arg GID="${GID}" \
        -t "${NAME}:${TAG}" .
else
    docker build \
        --build-arg UID="${UID}" \
        --build-arg GID="${GID}" \
        -t "${NAME}:${TAG}" .
fi