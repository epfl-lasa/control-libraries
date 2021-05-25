#!/bin/sh

docker build --tag control-libraries/python/test . || exit 1

docker run -it --rm \
  --name control-libraries-python-test \
  control-libraries/python/test
