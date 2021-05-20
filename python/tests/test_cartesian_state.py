#!/usr/bin/env python
from state_representation import CartesianState, dist
import numpy as np

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
    'inverse',
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


try:
    CartesianState()
    A = CartesianState("A")
    CartesianState(A)
    CartesianState("B", "C")

    A = CartesianState.Identity("A")
    B = CartesianState.Random("B")

    A.set_position(1, 2, 3)
    A.set_position([1, 2, 3])
    A.set_position(np.array([1, 2, 3]))

    A.set_orientation([1, 2, 3, 4])

    A.set_twist([1, 2, 3, 4, 5, 6])
    A.set_accelerations([1, 2, 3, 4, 5, 6])
    A.set_wrench([1, 2, 3, 4, 5, 6])

    print(A)

    A + B
    A - B
    A * 2
    2 * A
    A.inverse() * B
except Exception as e:
    print(f'Test failed: {e}')
    raise e
