#!/bin/sh -l

bash /github/workspace/source/install.sh --auto --no-controllers --no-dynamical-systems --no-robot-model || exit 1
ldconfig

echo ">>> Building Python bindings..."
pip3 install /github/workspace/python || (echo ">>> [ERROR] Build stage failed!" && exit 2) || exit $?

echo ">>> Running all test stages..."
python3 -m unittest discover /github/workspace/python --verbose \
  || (echo ">>> [ERROR] Test stage failed!" && exit 3) || exit $?

echo ">>> Test stages completed successfully!"

exit 0