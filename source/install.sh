#!/bin/bash
SCRIPT=$(readlink -f "$BASH_SOURCE")
CURRENT_PATH=$(dirname "$SCRIPT")

# turn on testing
IS_TEST="OFF"

# cpp
cd "${CURRENT_PATH}/state_representation" && . ./install.sh
cd "${CURRENT_PATH}/dynamical_systems" && . ./install.sh
cd "${CURRENT_PATH}/robot_model" && . ./install.sh
