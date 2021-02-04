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

MULTISTAGE_TARGET="development"

NAME=$(echo "${PWD##*/}" | tr _ -)/$MULTISTAGE_TARGET
TAG="latest"

MYUID="$(id -u "${USER}")"
MYGID="$(id -g "${USER}")"

if [ "$(uname -s)" = "Darwin" ]; then
    MYGID=1000
fi

if [ "$REBUILD" -eq 1 ]; then
    docker build \
        --no-cache \
        --target "${MULTISTAGE_TARGET}" \
        --build-arg UID="${MYUID}" \
        --build-arg GID="${MYGID}" \
        -t "${NAME}:${TAG}" .
else
    docker build \
        --target "${MULTISTAGE_TARGET}" \
        --build-arg UID="${MYUID}" \
        --build-arg GID="${MYGID}" \
        -t "${NAME}:${TAG}" .
fi
