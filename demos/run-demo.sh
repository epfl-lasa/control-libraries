#!/usr/bin/env bash

IMAGE_NAME=epfl-lasa/control-libraries/control-loop-examples
IMAGE_TAG=latest

HELP_MESSAGE="Usage: run-demo.sh [-s <script>] [-r] [-v]
Options:
  -s, --script             If provided, the desired script that should be
                           executed when starting the container.
  -r, --rebuild            Rebuild the image using the docker
                           --no-cache option.
  -v, --verbose            Use the verbose option during the building
                           process.
  -h, --help               Show this help message.
"

BUILD_FLAGS=()
TARGET_SCRIPT=""
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -s|--script) TARGET_SCRIPT="$2"; shift;;
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
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . -t "${IMAGE_NAME}:${IMAGE_TAG}" || exit 1

if [ -z "${TARGET_SCRIPT}" ]; then
  docker run -it --rm "${IMAGE_NAME}:${IMAGE_TAG}"
else
  docker run --rm "${IMAGE_NAME}:${IMAGE_TAG}" "./${TARGET_SCRIPT}"
fi
