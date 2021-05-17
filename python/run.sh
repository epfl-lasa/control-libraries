#!/bin/sh

docker build --tag control-libraries/python/test .

docker run -it --rm \
  --name control-libraries-python-test \
  control-libraries/python/test
