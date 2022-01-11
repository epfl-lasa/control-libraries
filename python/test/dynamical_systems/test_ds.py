import state_representation as sr
import unittest
from dynamical_systems import ICartesianDS, DYNAMICAL_SYSTEM, create_cartesian_ds, create_joint_ds

DS_METHOD_EXPECTS = [
    'is_compatible',
    'evaluate',
    'get_base_frame',
    'set_base_frame',
    'get_parameter',
    'get_parameters',
    'get_parameter_value',
    'get_parameter_list',
    'set_parameter',
    'set_parameters'
]


class TestDynamicalSystems(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(ICartesianDS) if callable(getattr(ICartesianDS, m))]
        for expected in DS_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_cartesian_factory(self):
        cart_ds = create_cartesian_ds(DYNAMICAL_SYSTEM.NONE)
        cart_ds.set_base_frame(sr.CartesianState.Identity("A"))
        cart_ds.evaluate(sr.CartesianState.Identity("C", "A"))
        self.assertTrue(cart_ds.evaluate(sr.CartesianState.Identity("C", "A")).is_empty())
        self.assertTrue(len(cart_ds.get_parameter_list()) == 0)

        param_list = [sr.Parameter("test", 1.0, sr.StateType.PARAMETER_DOUBLE)]
        with self.assertRaises(RuntimeError):
            cart_ds.set_parameters(param_list)

    def test_cartesian_ds(self):
        ds = create_cartesian_ds(DYNAMICAL_SYSTEM.POINT_ATTRACTOR)
        expected_params = ["attractor", "gain"]
        for param in list(ds.get_parameters().keys()):
            self.assertIn(param, expected_params)

        ds = create_cartesian_ds(DYNAMICAL_SYSTEM.RING)
        expected_params = ["center", "rotation_offset", "radius", "width", "speed", "field_strength", "normal_gain",
                           "angular_gain"]
        for param in list(ds.get_parameters().keys()):
            self.assertIn(param, expected_params)

        ds = create_cartesian_ds(DYNAMICAL_SYSTEM.CIRCULAR)
        expected_params = ["limit_cycle", "planar_gain", "normal_gain", "circular_velocity"]
        for param in list(ds.get_parameters().keys()):
            self.assertIn(param, expected_params)

    def test_joint_factory(self):
        joint_ds = create_joint_ds(DYNAMICAL_SYSTEM.NONE)
        joint_ds.set_base_frame(sr.JointState.Zero("robot", 3))
        joint_ds.evaluate(sr.JointState.Random("robot", 3))
        self.assertTrue(joint_ds.evaluate(sr.JointState.Random("robot", 3)).is_empty())
        self.assertTrue(len(joint_ds.get_parameter_list()) == 0)

        param_list = [sr.Parameter("test", 1.0, sr.StateType.PARAMETER_DOUBLE)]
        with self.assertRaises(RuntimeError):
            joint_ds.set_parameters(param_list)

    def test_joint_ds(self):
        ds = create_joint_ds(DYNAMICAL_SYSTEM.POINT_ATTRACTOR)
        expected_params = ["attractor", "gain"]
        for param in list(ds.get_parameters().keys()):
            self.assertIn(param, expected_params)


if __name__ == '__main__':
    unittest.main()
