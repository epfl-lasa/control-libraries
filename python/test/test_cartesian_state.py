import unittest
from state_representation import CartesianState, dist
import numpy as np

CARTESIAN_STATE_METHOD_EXPECTS = [
    'Identity',
    'Random',
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'set_data',
    'dist',
    'from_list',
    'get_accelerations',
    'get_angular_acceleration',
    'get_angular_velocity',
    'get_force',
    'get_linear_acceleration',
    'get_linear_velocity',
    'get_name',
    'get_orientation',
    'get_pose',
    'get_position',
    'get_reference_frame',
    'get_timestamp',
    'get_torque',
    'get_transformation_matrix',
    'get_twist',
    'get_type',
    'get_wrench',
    'initialize',
    'inverse',
    'is_compatible',
    'is_deprecated',
    'is_empty',
    'normalize',
    'normalized',
    'norms',
    'reset_timestamp',
    'set_accelerations',
    'set_angular_acceleration',
    'set_angular_velocity',
    'set_empty',
    'set_filled',
    'set_force',
    'set_linear_acceleration',
    'set_linear_velocity',
    'set_name',
    'set_orientation',
    'set_pose',
    'set_position',
    'set_reference_frame',
    'set_torque',
    'set_twist',
    'set_wrench',
    'set_zero',
    'to_list'
]

class TestCartesianState(unittest.TestCase):
    def assert_np_array_equal(self, a, b):
        self.assertListEqual(list(a), list(b))

    def assert_np_array_almost_equal(self, a, b):
        [self.assertAlmostEqual(x, y) for x, y in zip(list(a),list(b))]

    def test_callable_methods(self):
        methods = [m for m in dir(CartesianState) if callable(getattr(CartesianState, m))]
        for expected in CARTESIAN_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        CartesianState()
        A = CartesianState("A")
        CartesianState(A)
        CartesianState("B", "C")
        A = CartesianState.Identity("A")
        B = CartesianState.Random("B")

    def test_setters(self):
        A = CartesianState.Identity("A")

        A.set_position(1, 2, 3)
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])
        A.set_position([1, 2, 3])
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])
        A.set_position(np.array([1, 2, 3]))
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])

        A.set_orientation([1, 2, 3, 4])
        self.assert_np_array_equal(A.get_orientation(), [1, 2, 3, 4] / np.linalg.norm([1, 2, 3, 4]))

        A.set_twist([1, 2, 3, 4, 5, 6])
        self.assert_np_array_equal(A.get_twist(), [1, 2, 3, 4, 5, 6])
        A.set_accelerations([1, 2, 3, 4, 5, 6])
        self.assert_np_array_equal(A.get_accelerations(), [1, 2, 3, 4, 5, 6])
        A.set_wrench([1, 2, 3, 4, 5, 6])
        self.assert_np_array_equal(A.get_wrench(), [1, 2, 3, 4, 5, 6])

        data = [i for i in range(25)]
        data[3:7] = A.get_orientation().tolist()
        A.set_data(data)
        self.assert_np_array_almost_equal(A.data(), data)

    def test_operators(self):
        A = CartesianState.Random("A")
        B = CartesianState.Random("B")
        CinA = CartesianState.Random("C", "A")

        C = A * CinA
        A = A * 2
        A = 2 * A
        A *= 2

        A = A + B
        A += B

        A = A - B
        A -= B

if __name__ == '__main__':
    unittest.main()