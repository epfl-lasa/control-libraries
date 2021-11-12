#!/bin/sh -l

echo ">>> Installing control libraries..."
bash /github/workspace/source/install.sh --auto --no-controllers --no-dynamical-systems --no-robot-model || exit 1
bash /github/workspace/protocol/install.sh --auto || exit 1

echo ">>> Building Python bindings..."
pip3 install /github/workspace/python || (echo ">>> [ERROR] Build stage failed!" && exit 2) || exit $?

echo ">>> Running all test stages..."
python3 -m unittest discover /github/workspace/python --verbose \
  || (echo ">>> [ERROR] Test stage failed!" && exit 3) || exit $?

echo ">>> Test stages completed successfully!"

exit 0