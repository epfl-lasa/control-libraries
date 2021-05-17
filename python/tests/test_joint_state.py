#!/usr/bin/env python
from state_representation import JointState

methods = [m for m in dir(JointState) if callable(getattr(JointState, m))]
expected = [
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'dist',
    'from_list',
    'get_accelerations',
    'get_names',
    'get_positions',
    'get_size',
    'get_torques',
    'get_velocities',
    'set_accelerations',
    'set_names',
    'set_positions',
    'set_torques',
    'set_velocities',
    'set_zero',
    'to_list'
]

for method in expected:
    if method not in methods:
        raise AttributeError(f'Method {method} not defined!')
