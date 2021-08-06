#!/bin/sh -l

echo ">>> Installing control libraries..."
bash /github/workspace/source/install.sh --auto --no-controllers --no-dynamical-systems --no-robot-model || exit 1
ldconfig

echo ">>> Building proto bindings..."
cd /github/workspace/protocol/protobuf && make all || exit 2

echo ">>> Configuring clproto_cpp cmake..."
cd /github/workspace/protocol/clproto_cpp && mkdir build && cd build && cmake -DBUILD_TESTING=ON .. || \
  (echo ">>> [ERROR] Configuration stage failed!" && exit 3) || exit $?

echo ">>> Building clproto_cpp..."
make all || (echo ">>> [ERROR] Build stage failed!" && exit 4) || exit $?
echo ">>> Build stage completed successfully!"

echo ">>> Running all test stages..."
CTEST_OUTPUT_ON_FAILURE=1 make test || (echo ">>> [ERROR] Test stage failed!" && exit 5) || exit $?
echo ">>> Test stages completed successfully!"

exit 0