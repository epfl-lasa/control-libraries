#!/usr/bin/env python
from py_state_representation import CartesianState, dist

methods = [m for m in dir(CartesianState) if callable(getattr(CartesianState, m))]
expected = [
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'dist',
    'from_list',
    'get_accelerations',
    'get_angular_velocity',
    'get_force',
    'get_linear_acceleration',
    'get_linear_velocity',
    'get_orientation',
    'get_pose',
    'get_position',
    'get_torque',
    'get_transformation_matrix',
    'get_twist',
    'get_wrench',
    'normalize',
    'normalized',
    'norms',
    'set_accelerations',
    'set_angular_velocity',
    'set_force',
    'set_linear_acceleration',
    'set_linear_velocity',
    'set_orientation',
    'set_pose',
    'set_position',
    'set_torque',
    'set_twist',
    'set_wrench',
    'set_zero',
    'to_list'
]

for method in expected:
    if method not in methods:
        raise AttributeError(f'Method {method} not defined!')

A = CartesianState()
B = CartesianState("B")
C = CartesianState("C", "D")
D = CartesianState(C)

E = CartesianState.Identity("E")
F = CartesianState.Random("F")

print(F)
print(dist(E, F))
