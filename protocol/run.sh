#!/usr/bin/env bash

IMAGE_NAME=control-libraries/protobuf-development
STAGE_NAME=protobuf-development-dependencies
CONTAINER_NAME=control-libraries-protobuf
CONTAINER_HOSTNAME=control-libraries-protobuf

DOCKER_BUILDKIT=1 docker build -f ../Dockerfile --target "${STAGE_NAME}" --tag "${IMAGE_NAME}" ..

docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_HOSTNAME}" \
  "${IMAGE_NAME}" /bin/bash