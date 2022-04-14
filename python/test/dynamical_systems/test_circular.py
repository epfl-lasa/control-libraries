import numpy as np
import state_representation as sr
import unittest
from datetime import timedelta
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE


class TestCircular(unittest.TestCase):

    @classmethod
    def setUp(cls):
        cls.center = sr.CartesianPose.Identity("B")
        cls.limit_cycle = sr.Ellipsoid("limit_cycle")
        cls.radius = 10.0
        cls.nb_steps = 2000
        cls.dt = timedelta(milliseconds=10)
        cls.tol = 1e-3
        cls.limit_cycle.set_center_pose(cls.center)
        cls.limit_cycle.set_axis_lengths([cls.radius, cls.radius])

    def assert_np_array_equal(self, a: np.array, b: np.array, places=3):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def test_empty_constructor(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.CIRCULAR)
        self.assertTrue(ds.get_parameter_value("limit_cycle").get_center_state().is_empty())
        self.assertTrue(ds.get_base_frame().is_empty())

        ds.set_parameter(sr.Parameter("limit_cycle", self.limit_cycle, sr.ParameterType.STATE, sr.StateType.GEOMETRY_ELLIPSOID))
        self.assertFalse(ds.get_parameter_value("limit_cycle").get_center_state().is_empty())
        self.assertFalse(ds.get_base_frame().is_empty())

        self.assertEqual(ds.get_base_frame().get_name(), self.center.get_reference_frame())
        self.assertEqual(ds.get_base_frame().get_reference_frame(), self.center.get_reference_frame())
        self.assert_np_array_equal(ds.get_base_frame().get_transformation_matrix(), np.eye(4))

    def test_is_compatible(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.CIRCULAR)
        state1 = sr.CartesianState.Identity("world", "A")
        state2 = sr.CartesianState("D", "C")
        state3 = sr.CartesianState("C", "A")
        state4 = sr.CartesianState("C", "world")

        with self.assertRaises(RuntimeError):
            ds.evaluate(state1)

        ds.set_base_frame(state1)
        with self.assertRaises(RuntimeError):
            ds.evaluate(state2)
        with self.assertRaises(RuntimeError):
            ds.evaluate(state3)
        with self.assertRaises(RuntimeError):
            ds.evaluate(state4)

        ds.set_parameter(sr.Parameter("limit_cycle", self.limit_cycle, sr.ParameterType.STATE, sr.StateType.GEOMETRY_ELLIPSOID))
        self.assertTrue(ds.is_compatible(state1))
        self.assertFalse(ds.is_compatible(state2))
        self.assertTrue(ds.is_compatible(state3))
        self.assertTrue(ds.is_compatible(state4))

    def test_points_on_radius_random_center(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.CIRCULAR)
        self.limit_cycle.set_center_position(np.random.rand(3, 1))
        ds.set_parameter(sr.Parameter("limit_cycle", self.limit_cycle, sr.ParameterType.STATE, sr.StateType.GEOMETRY_ELLIPSOID))

        current_pose = sr.CartesianPose("A", 10 * np.random.rand(3, 1))
        for i in range(self.nb_steps):
            twist = sr.CartesianTwist(ds.evaluate(current_pose))
            twist.clamp(10, 10, 0.001, 0.001)
            current_pose += self.dt * twist

        self.assertTrue(current_pose.dist(self.limit_cycle.get_center_pose(),
                                          sr.CartesianStateVariable.POSITION) - self.radius < self.tol)


if __name__ == '__main__':
    unittest.main()
