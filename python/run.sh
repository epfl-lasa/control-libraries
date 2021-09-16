#!/bin/sh

BRANCH=$(git branch --show-current)
if [ -n "$1" ]; then
  BRANCH=$1
fi
echo "Using control libraries branch ${BRANCH}"

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest
docker build . --file ./Dockerfile.python \
  --build-arg BRANCH="${BRANCH}" \
  --target dev-user \
  --tag control-libraries/python/test || exit 1

docker run -it --rm \
  --volume "$(pwd)":/source/control-libraries/python \
  --name control-libraries-python-test \
  control-libraries/python/test
