#!/usr/bin/env bash

IMAGE_NAME=ghcr.io/epfl-lasa/control-libraries/development-dependencies
CONTAINER_NAME=epfl-lasa-control-libraries-development-dependencies-ssh

SSH_PORT=2222
SSH_KEY_FILE="${HOME}/.ssh/id_rsa.pub"
USERNAME=developer

HELP_MESSAGE="Usage: ./dev-server.sh [-p <port>] [-k <file>] [-u <user>]

Build and run a docker container as an SSH toolchain server for remote development.

The server is bound to the specified port on localhost (127.0.0.1)
and uses passwordless RSA key-pair authentication. The host public key
is read from the specified key file and copied to the server on startup.
On linux hosts, the UID and GID of the specified user will also be
set to match the UID and GID of the host user by the entry script.

The server will run in the background as ${CONTAINER_NAME}.

You can connect with 'ssh <user>@localhost -p <port>'.

Close the server with 'docker stop ${CONTAINER_NAME}'.

Options:
  -p, --port <XXXX>        Specify the port to bind for SSH
                           connection.
                           (default: ${SSH_PORT})

  -k, --key-file [path]    Specify the path of the RSA
                           public key file.
                           (default: ${SSH_KEY_FILE})

  -u, --user <user>        Specify the name of the remote user.
                           (default: ${USERNAME})

  -h, --help               Show this help message"

FWD_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
  -p | --port)
    # only capture the port argument for SSH
    # the first time, otherwise forward it
    if [ -z "${CUSTOM_SSH_PORT}" ]; then
      CUSTOM_SSH_PORT=$2
    else
      FWD_ARGS+=("$1 $2")
    fi
    shift 2
    ;;
  -k | --key-file) SSH_KEY_FILE=$2; shift 2;;
  -u | --user) USERNAME=$2; shift 2;;
  -h | --help) echo "${HELP_MESSAGE}"; exit 0;;
  *)
    echo 'Error in command line parsing' >&2
    echo -e "\n${HELP_MESSAGE}"
    exit 1
  esac
done

if [ -n "${CUSTOM_SSH_PORT}" ]; then
  SSH_PORT="${CUSTOM_SSH_PORT}"
fi

PUBLIC_KEY=$(cat "${SSH_KEY_FILE}")

COMMAND_FLAGS=()
COMMAND_FLAGS+=(--key "${PUBLIC_KEY}")
COMMAND_FLAGS+=(--user "${USERNAME}")

RUN_FLAGS=()
if [[ "${OSTYPE}" != "darwin"* ]]; then
  USER_ID=$(id -u "${USER}")
  GROUP_ID=$(id -g "${USER}")
  COMMAND_FLAGS+=(--uid "${USER_ID}")
  COMMAND_FLAGS+=(--gid "${GROUP_ID}")
fi

docker pull "${IMAGE_NAME}" || exit 1

docker container stop "${CONTAINER_NAME}" >/dev/null 2>&1
docker rm --force "${CONTAINER_NAME}" >/dev/null 2>&1

if [ ${#FWD_ARGS[@]} -gt 0 ]; then
  echo "Forwarding additional arguments to docker run command:"
  echo "${FWD_ARGS[@]}"
fi

echo "Starting background container with access port ${SSH_PORT} for user ${USERNAME}"
docker run -d --rm --cap-add sys_ptrace \
  --user root \
  --publish 127.0.0.1:"${SSH_PORT}":22 \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_NAME}" \
  "${RUN_FLAGS[@]}" \
  "${FWD_ARGS[@]}" \
  "${IMAGE_NAME}" /sshd_entrypoint.sh "${COMMAND_FLAGS[@]}"

echo "${CONTAINER_NAME}"