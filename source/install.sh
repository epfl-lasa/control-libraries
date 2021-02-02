#!/bin/bash
SCRIPT=$(readlink -f "$BASH_SOURCE")
SOURCE_PATH=$(dirname "$SCRIPT")

# turn on testing
IS_TEST="OFF"

# cpp
cd "${SOURCE_PATH}/state_representation" && . ./install.sh
cd "${SOURCE_PATH}/dynamical_systems" && . ./install.sh
cd "${SOURCE_PATH}/robot_model" && . ./install.sh
