#!/bin/sh -l

cd /github/workspace/source && mkdir build && cd build || exit 1

echo "Configuring cmake"
cmake -DBUILD_CONTROLLERS=ON -DBUILD_DYNAMICAL_SYSTEMS=ON -DBUILD_ROBOT_MODEL=ON -DBUILD_TESTING=ON ..

echo "Building project"
make -j all

echo "Running all test stages"
CTEST_OUTPUT_ON_FAILURE=1 make test

echo "Test stages completed successfully!"
exit 0