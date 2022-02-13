import unittest
import copy

import numpy as np
from state_representation import Jacobian, CartesianPose, CartesianTwist, CartesianWrench, JointVelocities

JACOBIAN_METHOD_EXPECTS = [
    'Random',
    'rows',
    'row',
    'cols',
    'col',
    'data',
    'get_joint_names',
    'get_frame',
    'get_reference_frame',
    'set_joint_names',
    'set_reference_frame',
    'set_data',
    'is_compatible',
    'initialize',
    'transpose',
    'inverse',
    'pseudoinverse',
    'solve',
    'copy',
]


class TestJacobian(unittest.TestCase):
    def assert_np_array_equal(self, a, b):
        self.assertListEqual(list(a), list(b))

    def assert_np_array_almost_equal(self, a, b):
        a = list(a)
        b = list(b)
        for i in range(len(a)):
            self.assertAlmostEqual(a[i], b[i])

    def test_callable_methods(self):
        methods = [m for m in dir(Jacobian) if callable(getattr(Jacobian, m))]
        for expected in JACOBIAN_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        A = Jacobian("robot", 7, "ee")
        Jacobian(A)
        Jacobian("robot", ["joint_0", "joint_1"], "ee", "robot")
        Jacobian.Random("robot", ["joint_0", "joint_1"], "ee", "robot")
        B = Jacobian.Random("robot", 7, "ee")
        B.copy()

    def test_copy(self):
        state = Jacobian.Random("robot", 7, "ee")
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assertEqual(state.get_frame(), state_copy.get_frame())
            self.assertEqual(state.get_reference_frame(), state_copy.get_reference_frame())
            self.assertListEqual(state.get_joint_names(), state_copy.get_joint_names())
            self.assert_np_array_equal(state.data().flatten(), state_copy.data().flatten())

    def test_getters(self):
        jac = Jacobian("jacobian", 3, "ee", "robot")
        self.assertEqual(jac.cols(), 3)
        jac.set_joint_names(["joint__0", "joint__1", "joint__2"])
        self.assertListEqual(jac.get_joint_names(), ["joint__0", "joint__1", "joint__2"])
        self.assertEqual(jac.get_frame(), "ee")
        self.assertEqual(jac.get_reference_frame(), "robot")

        data = np.random.rand(6, 3)
        jac.set_data(data)
        self.assert_np_array_equal(jac.col(1), data[:, 1])
        self.assert_np_array_equal(jac.row(1), data[1, :])

        jac = jac.transpose()
        self.assertEqual(jac.rows(), 3)

    def test_solve(self):
        jac = Jacobian.Random("jac", 7, "ee", "robot")
        twist = CartesianTwist.Random("ee", "robot")
        jac.solve(twist)

        data = np.random.rand(6, 7)
        jac.solve(data)

    def test_operators(self):
        jac = Jacobian.Random("test", 3, "ee", "robot")
        elem = jac[1, 2]
        jac[1, 2] = 0.1

        matrix = np.random.rand(3, 6)
        result = jac * matrix
        result = matrix * jac

        joint_velocities = JointVelocities.Random("test", 3)
        twist = jac * joint_velocities
        self.assert_np_array_almost_equal(joint_velocities.get_velocities(),
                                          (jac.pseudoinverse() * twist).get_velocities())

        wrench = CartesianWrench.Random("ee", "robot")
        torques = jac.transpose() * wrench

        pose = CartesianPose.Random("robot", "world")
        jac_in_world = pose * jac


if __name__ == '__main__':
    unittest.main()
