#!/bin/bash
ROS_VERSION=noetic

IMAGE_NAME=control-libraries/ros-demos:latest

BUILD_FLAGS=()
BRANCH=develop
while [ "$#" -gt 0 ]; do
  case "$1" in
  -r|--rebuild)
    BUILD_FLAGS+=(--no-cache)
    shift 1
    ;;
  -b|--branch)
    BRANCH=$2
    shift 2
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 1
    ;;
  esac
done

docker pull ghcr.io/aica-technology/ros-ws:"${ROS_VERSION}"

echo "Using control libraries branch ${BRANCH}"

BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(--build-arg BRANCH="${BRANCH}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .
