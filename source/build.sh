#!/bin/bash
CURRPATH=${PWD}
# turn on testing
IS_TEST="OFF"

# cpp
cd "${CURRPATH}/state_representation" && mkdir -p build && cd build && cmake -Druntests="${IS_TEST}" .. && make -j && sudo make install
cd "${CURRPATH}/dynamical_systems" && mkdir -p build && cd build && cmake -Druntests="${IS_TEST}" .. && make -j && sudo make install
cd "${CURRPATH}/robot_model" && mkdir -p build && cd build && cmake -Druntests="${IS_TEST}" .. && make -j && sudo make install
