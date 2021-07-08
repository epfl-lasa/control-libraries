#!/bin/sh

BRANCH=$(git branch --show-current)
if [ -n "$1" ]; then
  BRANCH=$1
fi
echo "Using control libraries branch ${BRANCH}"

docker build --build-arg BRANCH="${BRANCH}" --tag control-libraries/python/test . || exit 1

docker run -it --rm \
  --volume "$(pwd)":/source/control_libraries/python \
  --name control-libraries-python-test \
  control-libraries/python/test
