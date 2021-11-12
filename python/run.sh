#!/bin/sh

BRANCH=$(git branch --show-current)

BUILD_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    -b|--branch) BRANCH=$2; shift 2;;
    -r|--rebuild) BUILD_ARGS+=(--no-cache); shift 1;;
    *) echo "Unknown option: $1" >&2; exit 1;;
  esac
done

echo "Using control libraries branch ${BRANCH}"
BUILD_ARGS+=(--build-arg BRANCH="${BRANCH}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest
docker build . --file ./Dockerfile.python \
  "${BUILD_ARGS[@]}" \
  --target dev-user \
  --tag control-libraries/python/test || exit 1

docker run -it --rm \
  --volume "$(pwd)":/source/control-libraries/python \
  --name control-libraries-python-test \
  control-libraries/python/test
