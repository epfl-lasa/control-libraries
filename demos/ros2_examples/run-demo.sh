#!/usr/bin/env bash

# change to  false to use host network
IS_ISOLATED=true

MULTISTAGE_TARGET="runtime-demonstrations"
IMAGE_NAME=epfl-lasa/control-libraries/ros2-examples
IMAGE_TAG="${MULTISTAGE_TARGET}"
GPUS=""

NETWORK=host
if [ "${IS_ISOLATED}" = true ]; then
    docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
    NETWORK=isolated
fi

HELP_MESSAGE="Usage: run-demo.sh [-r] [-v]
Options:
  --gpus <gpu_options>     Add GPU access for applications that
                           require hardware acceleration (e.g. Gazebo)
                           For the list of gpu_options parameters see:
      >>> https://docs.docker.com/config/containers/resource_constraints
  -r, --rebuild            Rebuild the image using the docker
                           --no-cache option.
  -v, --verbose            Use the verbose option during the building
                           process.
  -h, --help               Show this help message.
"

BUILD_FLAGS=()
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    --gpus) GPUS=$2; shift 1;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies
DOCKER_BUILDKIT=1 docker build --target install \
  -t epfl-lasa/control-libraries/source:install \
  --build-arg BUILD_TESTING=OFF \
  "${BUILD_FLAGS[@]}" \
  -f ../../source/Dockerfile.source ../../source || exit 1
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . --target "${MULTISTAGE_TARGET}" -t "${IMAGE_NAME}:${IMAGE_TAG}" || exit 1

RUN_FLAGS=(-u developer)
if [ -n "${GPUS}" ]; then
  RUN_FLAGS+=(--gpus "${GPUS}")
  RUN_FLAGS+=(--env DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(--env NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}")
  RUN_FLAGS+=(--env NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics")
fi

if [[ "${OSTYPE}" == "darwin"* ]]; then
  RUN_FLAGS+=(-e DISPLAY=host.docker.internal:0)
else
  xhost +
  RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
  RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  RUN_FLAGS+=(--device=/dev/dri:/dev/dri)
fi

docker run -it --rm --net="${NETWORK}" \
  "${RUN_FLAGS[@]}" \
  --volume="$(pwd)/rviz:/home/developer/ros2_ws/src/ros2_examples/rviz/:rw" \
  --name "${CONTAINER_NAME}" \
  "${IMAGE_NAME}:${IMAGE_TAG}" /bin/bash
