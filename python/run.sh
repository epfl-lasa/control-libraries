#!/bin/sh

docker build --tag control-libraries/python/test .

LOCAL_PATH=$(pwd)
docker run -it --rm \
  -v "$LOCAL_PATH":/home/dev \
  --name control-libraries-python-test \
  control-libraries/python/test
