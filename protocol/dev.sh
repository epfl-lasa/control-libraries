#!/usr/bin/env bash
# Build and run a docker container as an SSH toolchain server for remote development

IMAGE_NAME=control-libraries/source-dependencies
CONTAINER_NAME=control-libraries-source-dependencies-protobuf

DOCKER_BUILDKIT=1 docker build -f ../Dockerfile --target source-dependencies --tag "${IMAGE_NAME}" ..

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

docker run -it --rm \
  -v "${PWD}":/home/remote/protocol \
  --cap-add sys_ptrace -p127.0.0.1:2222:22 \
  --name "${CONTAINER_NAME}" "${IMAGE_NAME}"
