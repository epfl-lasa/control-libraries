#!/usr/bin/env bash

IMAGE_NAME=control-libraries/remote-development
STAGE_NAME=remote-development
CONTAINER_NAME=control-libraries-remote-development-ssh
CONTAINER_HOSTNAME=control-libraries-remote-development

SSH_PORT=2222
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
  -p, --port [XXXX]        Specify the port to bind for SSH
                           connection.
                           (default: ${SSH_PORT})

  -k, --key-file [path]    Specify the path of the RSA
                           public key file.
                           (default: ${SSH_KEY_FILE})

  -h, --help               Show this help message."

while [ "$#" -gt 0 ]; do
  case "$1" in
    -p|--port) SSH_PORT=$2; shift 2;;
    -k|--key-file) SSH_KEY_FILE=$2; shift 2;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;
    *) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies
DOCKER_BUILDKIT=1 docker build ./source --file ./source/Dockerfile --target "${STAGE_NAME}" --tag "${IMAGE_NAME}"

docker container stop "${CONTAINER_NAME}" >/dev/null 2>&1
docker rm --force "${CONTAINER_NAME}" >/dev/null 2>&1

docker run -d --cap-add sys_ptrace \
  --publish 127.0.0.1:"${SSH_PORT}":22 \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_HOSTNAME}" \
  "${IMAGE_NAME}" "$(cat "${SSH_KEY_FILE}")"