#!/usr/bin/env bash
# Build and run a docker container as an SSH toolchain server for remote development

IMAGE_NAME=control-libraries/remote-development
CONTAINER_NAME=control-libraries-remote-development-ssh

DOCKER_BUILDKIT=1 docker build -f Dockerfile --target remote-development --tag "${IMAGE_NAME}" .

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

docker run -d --cap-add sys_ptrace -p127.0.0.1:2222:22 --name "${CONTAINER_NAME}" "${IMAGE_NAME}"