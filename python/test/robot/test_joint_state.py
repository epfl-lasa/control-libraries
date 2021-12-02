import unittest
import copy

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
    def assert_np_array_equal(self, a, b):
        self.assertListEqual(list(a), list(b))

    def test_callable_methods(self):
        methods = [m for m in dir(JointState) if callable(getattr(JointState, m))]
        for expected in JOINT_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_copy(self):
        state = JointState().Random("test", 3)
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assertEqual(state.get_name(), state_copy.get_name())
            self.assertListEqual(state.get_names(), state_copy.get_names())
            self.assert_np_array_equal(state.data(), state_copy.data())

if __name__ == '__main__':
    unittest.main()
