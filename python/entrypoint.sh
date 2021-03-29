#!/usr/bin/env sh

cd /home/dev || exit
mkdir -p build
pipenv run bash -c "cd build && cmake .. && make DESTDIR=/home/dev install"

pipenv run python tests/test_cartesian_state.py
pipenv run python tests/test_joint_state.py

pipenv shell
