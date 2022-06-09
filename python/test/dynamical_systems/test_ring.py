import copy
import numpy as np
import state_representation as sr
import unittest
from datetime import timedelta
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE


class TestRing(unittest.TestCase):

    @classmethod
    def setUp(cls):
        cls.center = sr.CartesianPose.Identity("A")
        cls.radius = 10.0
        cls.width = 1.0
        cls.speed = 1.0
        cls.nb_steps = 1000
        cls.dt = timedelta(milliseconds=10)
        cls.tol = 1e-3

    def assert_np_array_equal(self, a: np.array, b: np.array, places=3):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def test_empty_constructor(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.RING)
        center = sr.CartesianPose.Identity("CAttractor", "A")

        self.assertTrue(ds.get_parameter_value("center").is_empty())
        ds.set_parameter(sr.Parameter("center", center, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))
        self.assertFalse(ds.get_parameter_value("center").is_empty())

        self.assertFalse(ds.get_base_frame().is_empty())
        self.assertEqual(ds.get_base_frame().get_name(), center.get_reference_frame())
        self.assertEqual(ds.get_base_frame().get_reference_frame(), center.get_reference_frame())
        self.assert_np_array_equal(ds.get_base_frame().get_transformation_matrix(), np.eye(4))

    def test_is_compatible(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.RING)
        center = sr.CartesianPose.Identity("CAttractor", "A")
        state1 = sr.CartesianState.Identity("B", "A")
        state2 = sr.CartesianState.Identity("D", "C")
        state3 = sr.CartesianState.Identity("C", "A")
        state4 = sr.CartesianState.Identity("C", "B")

        with self.assertRaises(RuntimeError):
            ds.evaluate(state1)

        ds.set_base_frame(state1)
        with self.assertRaises(RuntimeError):
            ds.evaluate(state2)
        with self.assertRaises(RuntimeError):
            ds.evaluate(state3)
        with self.assertRaises(RuntimeError):
            ds.evaluate(state4)

        ds.set_parameter(sr.Parameter("center", center, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))
        self.assertTrue(ds.is_compatible(state1))
        self.assertFalse(ds.is_compatible(state2))
        self.assertTrue(ds.is_compatible(state3))
        self.assertTrue(ds.is_compatible(state4))

    def test_points_on_radius(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.RING)
        ds.set_parameter(sr.Parameter("center", self.center, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))
        ds.set_parameter(sr.Parameter("radius", self.radius, sr.ParameterType.DOUBLE))
        ds.set_parameter(sr.Parameter("width", self.width, sr.ParameterType.DOUBLE))
        ds.set_parameter(sr.Parameter("speed", self.speed, sr.ParameterType.DOUBLE))

        current_pose = copy.deepcopy(self.center)
        twist = sr.CartesianTwist(ds.evaluate(current_pose))
        self.assertTrue(np.linalg.norm(twist.data()) < self.tol)

        current_pose.set_position(self.radius, 0, 0)
        twist = sr.CartesianTwist(ds.evaluate(current_pose))
        self.assertTrue(twist.get_linear_velocity()[0] < self.tol)
        self.assertTrue(abs(twist.get_linear_velocity()[1] - self.speed) < self.tol)

        current_pose.set_position(0, self.radius, 0)
        twist = sr.CartesianTwist(ds.evaluate(current_pose))
        self.assertTrue(abs(twist.get_linear_velocity()[0] + self.speed) < self.tol)
        self.assertTrue(twist.get_linear_velocity()[1] < self.tol)

        current_pose.set_position(-self.radius, 0, 0)
        twist = sr.CartesianTwist(ds.evaluate(current_pose))
        self.assertTrue(twist.get_linear_velocity()[0] < self.tol)
        self.assertTrue(abs(twist.get_linear_velocity()[1] + self.speed) < self.tol)

        current_pose.set_position(0, -self.radius, 0)
        twist = sr.CartesianTwist(ds.evaluate(current_pose))
        self.assertTrue(abs(twist.get_linear_velocity()[0] - self.speed) < self.tol)
        self.assertTrue(twist.get_linear_velocity()[1] < self.tol)

        current_pose.set_position(self.radius, 0, 1)
        twist = sr.CartesianTwist(ds.evaluate(current_pose))
        self.assertTrue(abs(twist.get_linear_velocity()[2] + 1) < self.tol)

    def test_convergence_on_radius_random_center(self):
        center = sr.CartesianPose.Random("A")
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.RING)
        ds.set_parameter(sr.Parameter("center", center, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))
        ds.set_parameter(sr.Parameter("radius", self.radius, sr.ParameterType.DOUBLE))

        current_pose = sr.CartesianPose("B", self.radius * np.random.rand(3, 1))
        for i in range(self.nb_steps):
            twist = sr.CartesianTwist(ds.evaluate(current_pose))
            twist.clamp(10, 10, 0.001, 0.001)
            current_pose += self.dt * twist

        self.assertTrue(
            abs(np.linalg.norm(current_pose.get_position() - center.get_position()) - self.radius) < self.tol)


if __name__ == '__main__':
    unittest.main()
