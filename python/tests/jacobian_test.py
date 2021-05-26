import unittest
from state_representation import Jacobian, CartesianTwist, CartesianWrench, JointVelocities
import numpy as np

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
    'set_rows',
    'set_cols',
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
        B.copy() # TODO does this actually give a copy?

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

        data = np.random.rand(6,7)
        jac.solve(data)

    def test_operators(self):
        matrix = np.random.rand(3, 6)
        jac = Jacobian.Random("test", 3, "ee", "robot")
        wrench = CartesianWrench.Random("ee", "robot")
        joint_velocities = JointVelocities.Random("test", 3)

        matrix = jac * matrix
        # matrix = jac.transpose() * jac # TODO
        # twist = jac.transpose() * joint_velocities

        # joint_velocities = jac * twist
        # joint_torques = jac * wrench


if __name__ == '__main__':
    unittest.main()