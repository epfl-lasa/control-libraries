import unittest
import state_representation as sr

import numpy as np


class TestParameters(unittest.TestCase):

    def assert_np_array_equal(self, a: np.array, b: np.array, places=3):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def cartesian_equal(self, cartesian1, cartesian2):
        self.assertTrue(cartesian1.get_name(), cartesian2.get_name())
        self.assertTrue(cartesian1.get_reference_frame(), cartesian2.get_reference_frame())
        self.assert_np_array_equal(cartesian1.data(), cartesian2.data())

    def joint_equal(self, joint1, joint2):
        self.assertTrue(joint1.get_name(), joint2.get_name())
        self.assertTrue(joint1.get_size(), joint2.get_size())
        self.assert_np_array_equal(joint1.data(), joint2.data())

    def test_param_invalid(self):
        param = sr.Parameter("test", sr.ParameterType.STRING)
        self.assertRaises(RuntimeError, param.set_value, 1)

    def test_param_copy(self):
        param = sr.Parameter("test", 1, sr.ParameterType.INT)
        self.assertEqual(param.get_name(), "test")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.INT)
        self.assertEqual(param.get_value(), 1)
        param_copy = sr.Parameter(param)
        self.assertEqual(param_copy.get_name(), "test")
        self.assertEqual(param_copy.get_parameter_type(), sr.ParameterType.INT)
        self.assertEqual(param_copy.get_value(), 1)

    def test_param_int(self):
        param = sr.Parameter("int", sr.ParameterType.INT)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "int")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.INT)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), 0)
        param.set_value(1)
        self.assertEqual(param.get_value(), 1)
        param1 = sr.Parameter("int", 1, sr.ParameterType.INT)
        self.assertEqual(param1.get_value(), 1)

    def test_param_int_array(self):
        param = sr.Parameter("int_array", sr.ParameterType.INT_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "int_array")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.INT_ARRAY)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), [])
        values = [2, 3, 4]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("int_array", values, sr.ParameterType.INT_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_double(self):
        param = sr.Parameter("double", sr.ParameterType.DOUBLE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "double")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.DOUBLE)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), 0.0)
        param.set_value(1.5)
        self.assertEqual(param.get_value(), 1.5)
        param1 = sr.Parameter("double", 1.5, sr.ParameterType.DOUBLE)
        self.assertEqual(param1.get_value(), 1.5)

    def test_param_double_array(self):
        param = sr.Parameter("double_array", sr.ParameterType.DOUBLE_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "double_array")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.DOUBLE_ARRAY)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), [])
        values = [2.2, 3.3, 4.4]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("double_array", values, sr.ParameterType.DOUBLE_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_bool(self):
        param = sr.Parameter("bool", sr.ParameterType.BOOL)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "bool")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.BOOL)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), False)
        param.set_value(True)
        self.assertEqual(param.get_value(), True)
        param1 = sr.Parameter("bool", True, sr.ParameterType.BOOL)
        self.assertEqual(param1.get_value(), True)

    def test_param_bool_array(self):
        param = sr.Parameter("bool_array", sr.ParameterType.BOOL_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "bool_array")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.BOOL_ARRAY)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), [])
        values = [True, False, False]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("bool_array", values, sr.ParameterType.BOOL_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_string(self):
        param = sr.Parameter("string", sr.ParameterType.STRING)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "string")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STRING)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), "")
        param.set_value("parameter")
        self.assertEqual(param.get_value(), "parameter")
        param1 = sr.Parameter("string", "parameter", sr.ParameterType.STRING)
        self.assertEqual(param1.get_value(), "parameter")

    def test_param_string_array(self):
        param = sr.Parameter("string_array", sr.ParameterType.STRING_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "string_array")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STRING_ARRAY)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assertEqual(param.get_value(), [])
        values = ["test", "parameter", "bindings"]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("string_array", values, sr.ParameterType.STRING_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_cartesian_state(self):
        param = sr.Parameter("cartesian_state", sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "cartesian_state")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STATE)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.CARTESIAN_STATE)
        self.assertTrue(param.get_value().is_empty())
        self.assertEqual(param.get_value().get_type(), sr.StateType.CARTESIAN_STATE)
        values = sr.CartesianState.Random("test")
        param.set_value(values)
        self.cartesian_equal(param.get_value(), values)
        param1 = sr.Parameter("cartesian_state", values, sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE)
        self.cartesian_equal(param1.get_value(), values)

    def test_param_cartesian_pose(self):
        param = sr.Parameter("cartesian_pose", sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "cartesian_pose")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STATE)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.CARTESIAN_POSE)
        self.assertTrue(param.get_value().is_empty())
        self.assertEqual(param.get_value().get_type(), sr.StateType.CARTESIAN_POSE)
        values = sr.CartesianPose.Random("test")
        param.set_value(values)
        self.cartesian_equal(param.get_value(), values)
        param1 = sr.Parameter("cartesian_pose", values, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE)
        self.cartesian_equal(param1.get_value(), values)

    def test_param_joint_state(self):
        param = sr.Parameter("joint_state", sr.ParameterType.STATE, sr.StateType.JOINT_STATE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "joint_state")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STATE)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.JOINT_STATE)
        self.assertTrue(param.get_value().is_empty())
        self.assertEqual(param.get_value().get_type(), sr.StateType.JOINT_STATE)
        values = sr.JointState.Random("test", 3)
        param.set_value(values)
        self.joint_equal(param.get_value(), values)
        param1 = sr.Parameter("joint_state", values, sr.ParameterType.STATE, sr.StateType.JOINT_STATE)
        self.joint_equal(param1.get_value(), values)

    def test_param_joint_positions(self):
        param = sr.Parameter("joint_positions", sr.ParameterType.STATE, sr.StateType.JOINT_POSITIONS)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "joint_positions")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STATE)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.JOINT_POSITIONS)
        print(param.get_value())
        self.assertTrue(param.get_value().is_empty())
        self.assertEqual(param.get_value().get_type(), sr.StateType.JOINT_POSITIONS)
        values = sr.JointPositions.Random("test", 3)
        param.set_value(values)
        self.joint_equal(param.get_value(), values)
        param1 = sr.Parameter("joint_positions", values, sr.ParameterType.STATE, sr.StateType.JOINT_POSITIONS)
        self.joint_equal(param1.get_value(), values)

    def test_param_ellipsoid(self):
        param = sr.Parameter("ellipse", sr.ParameterType.STATE, sr.StateType.GEOMETRY_ELLIPSOID)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "ellipse")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.STATE)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.GEOMETRY_ELLIPSOID)
        self.assertTrue(param.get_value().is_empty())
        self.assertEqual(param.get_value().get_type(), sr.StateType.GEOMETRY_ELLIPSOID)
        values = sr.Ellipsoid("test")
        param.set_value(values)
        self.assertTrue(param.get_value().get_name(), values.get_name())
        param1 = sr.Parameter("ellipse", values, sr.ParameterType.STATE, sr.StateType.GEOMETRY_ELLIPSOID)
        self.assertTrue(param1.get_value().get_name(), values.get_name())

    def test_param_matrix(self):
        param = sr.Parameter("matrix", sr.ParameterType.MATRIX)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "matrix")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.MATRIX)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assert_np_array_equal(param.get_value(), np.empty((0, 0)))
        values = np.random.rand(3, 2)
        param.set_value(values)
        self.assert_np_array_equal(param.get_value(), values)
        param1 = sr.Parameter("matrix", values, sr.ParameterType.MATRIX)
        self.assert_np_array_equal(param1.get_value(), values)

    def test_param_vector(self):
        param = sr.Parameter("vector", sr.ParameterType.VECTOR)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "vector")
        self.assertEqual(param.get_parameter_type(), sr.ParameterType.VECTOR)
        self.assertEqual(param.get_parameter_state_type(), sr.StateType.NONE)
        self.assert_np_array_equal(param.get_value(), np.empty(0))
        values = np.random.rand(3)
        param.set_value(values)
        self.assert_np_array_equal(param.get_value(), values)
        param1 = sr.Parameter("vector", values, sr.ParameterType.VECTOR)
        self.assert_np_array_equal(param1.get_value(), values)

    def param_map_equal(self, param_dict, param_map):
        def simple_param_equal(param1, param2):
            self.assertEqual(param1.get_name(), param2.get_name())
            self.assertEqual(param1.get_type(), param2.get_type())
            self.assertEqual(param1.get_value(), param2.get_value())

        for name, param in param_dict.items():
            p = param_map.get_parameter(name)
            simple_param_equal(p, param)

            value = param_map.get_parameter_value(name)
            self.assertEqual(value, param.get_value())

        for n, p in param_map.get_parameters().items():
            simple_param_equal(p, param_dict[n])

        for p in param_map.get_parameter_list():
            simple_param_equal(p, param_dict[p.get_name()])


    def test_param_map(self):
        param_dict = {"int": sr.Parameter("int", 1, sr.ParameterType.INT),
                      "double": sr.Parameter("double", 2.2, sr.ParameterType.DOUBLE),
                      "string": sr.Parameter("string", "test", sr.ParameterType.STRING)}
        param_list = [param for name, param in param_dict.items()]

        m = sr.ParameterMap()
        for name, param in param_dict.items():
            m.set_parameter(param)
        self.param_map_equal(param_dict, m)

        m = sr.ParameterMap()
        for name, param in param_dict.items():
            m.set_parameter_value(param.get_name(), param.get_value(), param.get_parameter_type(), param.get_parameter_state_type())
        self.param_map_equal(param_dict, m)

        m = sr.ParameterMap()
        m.set_parameters(param_dict)
        self.param_map_equal(param_dict, m)

        m = sr.ParameterMap()
        m.set_parameters(param_list)
        self.param_map_equal(param_dict, m)

        m = sr.ParameterMap(param_dict)
        self.param_map_equal(param_dict, m)

        m = sr.ParameterMap(param_list)
        self.param_map_equal(param_dict, m)


if __name__ == '__main__':
    unittest.main()
