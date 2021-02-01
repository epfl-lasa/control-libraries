#!/bin/bash
CURRPATH=${PWD}
# turn on testing
IS_TEST="OFF"

# cpp
cd "${CURRPATH}/state_representation" && . ./install.sh
cd "${CURRPATH}/dynamical_systems" && . ./install.sh
cd "${CURRPATH}/robot_model" && . ./install.sh
