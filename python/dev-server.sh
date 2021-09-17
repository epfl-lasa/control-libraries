#!/bin/bash

BASE_IMAGE=control-libraries/remote-development:latest
IMAGE_NAME=control-libraries/python/remote
CONTAINER_NAME=control-libraries-python-ssh
CONTAINER_HOSTNAME=control-libraries-python
BRANCH=$(git branch --show-current)

SSH_PORT=2233
SSH_KEY_FILE="$HOME/.ssh/id_rsa.pub"

HELP_MESSAGE="Usage: ./dev-server.sh [--port <port>] [--key-file </path/to/id_rsa.pub>]

Build and run a docker container as an SSH toolchain server for remote development.

The server is bound to the specified port on localhost (127.0.0.1)
and uses passwordless RSA key-pair authentication. The host public key
is read from the specified key file and copied to the server on startup.

The server will run in the background as ${CONTAINER_NAME}.

You can connect with 'ssh remote@localhost -p <port>'.

Close the server with 'docker container stop ${CONTAINER_NAME}'.

Options:
  -b, --branch [branch]    Specify the branch of control libraries
                           that should be used to build the image.
  -p, --port [XXXX]        Specify the port to bind for SSH
                           connection.
                           (default: ${SSH_PORT})

  -k, --key-file [path]    Specify the path of the RSA
                           public key file.
                           (default: ${SSH_KEY_FILE})

  -h, --help               Show this help message."

function image_not_found() {
  MESSAGE="Could not find the required Docker image '${BASE_IMAGE}'
from ../source/Dockerfile.source. Make sure to build it with the 'dev-server.sh' script
in the 'source' directory."
  echo "${MESSAGE}" && exit 1
}

docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1 || image_not_found

while [ "$#" -gt 0 ]; do
  case "$1" in
    -b|--branch) BRANCH=$2; shift 2;;
    -p|--port) SSH_PORT=$2; shift 2;;
    -k|--key-file) SSH_KEY_FILE=$2; shift 2;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;
    *) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done

echo "Using control libraries branch ${BRANCH}"

DOCKER_BUILDKIT=1 docker build . --file ./Dockerfile.python \
  --build-arg BASE_IMAGE=control-libraries/remote-development:latest \
  --build-arg BRANCH="${BRANCH}" \
  --target remote-user \
  --tag "${IMAGE_NAME}" || exit 1

docker container stop "${CONTAINER_NAME}" >/dev/null 2>&1
docker rm --force "${CONTAINER_NAME}" >/dev/null 2>&1

docker run -d --cap-add sys_ptrace \
  --publish 127.0.0.1:"${SSH_PORT}":22 \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_HOSTNAME}" \
  "${IMAGE_NAME}" "$(cat "${SSH_KEY_FILE}")"

echo "${CONTAINER_NAME}"
