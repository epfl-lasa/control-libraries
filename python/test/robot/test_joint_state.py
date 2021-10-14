import unittest
from state_representation import JointState

JOINT_STATE_METHOD_EXPECTS = [
    'Random',
    'Zero',
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'set_data',
    'dist',
    'get_accelerations',
    'get_acceleration',
    'get_joint_index',
    'get_name',
    'get_names',
    'get_positions',
    'get_position',
    'get_size',
    'get_timestamp',
    'get_torques',
    'get_torque',
    'get_type',
    'get_velocities',
    'get_velocity',
    'initialize',
    'is_compatible',
    'is_deprecated',
    'is_empty',
    'reset_timestamp',
    'set_accelerations',
    'set_empty',
    'set_filled',
    'set_name',
    'set_names',
    'set_positions',
    'set_position',
    'set_velocities',
    'set_velocity',
    'set_accelerations',
    'set_acceleration',
    'set_torques',
    'set_torque',
    'set_zero',
    'to_list'
]


class TestJointState(unittest.TestCase):
    def test_callable_methods(self):
        methods = [m for m in dir(JointState) if callable(getattr(JointState, m))]
        for expected in JOINT_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

if __name__ == '__main__':
    unittest.main()
