import unittest
import state_representation as sr

import numpy as np
from numpy.testing import assert_array_equal


class TestParameters(unittest.TestCase):

    def test_param_int(self):
        param = sr.Parameter("int", sr.StateType.PARAMETER_INT)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "int")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_INT)
        param.set_value(1)
        self.assertEqual(param.get_value(), 1)

    def test_param_int_array(self):
        param = sr.Parameter("int_array", sr.StateType.PARAMETER_INT_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "int_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_INT_ARRAY)
        values = [2, 3, 4]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_double(self):
        param = sr.Parameter("double", sr.StateType.PARAMETER_DOUBLE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "double")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_DOUBLE)
        param.set_value(1.5)
        self.assertEqual(param.get_value(), 1.5)

    def test_param_double_array(self):
        param = sr.Parameter("double_array", sr.StateType.PARAMETER_DOUBLE_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "double_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_DOUBLE_ARRAY)
        values = [2.2, 3.3, 4.4]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_bool(self):
        param = sr.Parameter("bool", sr.StateType.PARAMETER_BOOL)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "bool")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_BOOL)
        param.set_value(False)
        self.assertEqual(param.get_value(), False)

    def test_param_bool_array(self):
        param = sr.Parameter("bool_array", sr.StateType.PARAMETER_BOOL_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "bool_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_BOOL_ARRAY)
        values = [True, False, False]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_string(self):
        param = sr.Parameter("string", sr.StateType.PARAMETER_STRING)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "string")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_STRING)
        param.set_value("parameter")
        self.assertEqual(param.get_value(), "parameter")

    def test_param_string_array(self):
        param = sr.Parameter("string_array", sr.StateType.PARAMETER_STRING_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "string_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_STRING_ARRAY)
        values = ["test", "parameter", "bindings"]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_cartesian_state(self):
        param = sr.Parameter("cartesian_state", sr.StateType.PARAMETER_CARTESIANSTATE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "cartesian_state")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_CARTESIANSTATE)
        values = sr.CartesianState.Random("test")
        param.set_value(values)
        param_value = param.get_value()
        self.assertTrue(param_value.get_name(), values.get_name())
        self.assertTrue(param_value.get_reference_frame(), values.get_reference_frame())
        assert_array_equal(param_value.data(), values.data())

    def test_param_cartesian_pose(self):
        param = sr.Parameter("cartesian_pose", sr.StateType.PARAMETER_CARTESIANPOSE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "cartesian_pose")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_CARTESIANPOSE)
        values = sr.CartesianPose.Random("test")
        param.set_value(values)
        param_value = param.get_value()
        self.assertTrue(param_value.get_name(), values.get_name())
        self.assertTrue(param_value.get_reference_frame(), values.get_reference_frame())
        assert_array_equal(param_value.data(), values.data())

    def test_param_joint_state(self):
        param = sr.Parameter("joint_state", sr.StateType.PARAMETER_JOINTSTATE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "joint_state")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_JOINTSTATE)
        values = sr.JointState.Random("test", 3)
        param.set_value(values)
        param_value = param.get_value()
        self.assertTrue(param_value.get_name(), values.get_name())
        self.assertTrue(param_value.get_size(), values.get_size())
        assert_array_equal(param_value.data(), values.data())

    def test_param_cartesian_pose(self):
        param = sr.Parameter("joint_positions", sr.StateType.PARAMETER_JOINTPOSITIONS)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "joint_positions")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_JOINTPOSITIONS)
        values = sr.JointPositions.Random("test", 3)
        param.set_value(values)
        param_value = param.get_value()
        self.assertTrue(param_value.get_name(), values.get_name())
        self.assertTrue(param_value.get_size(), values.get_size())
        assert_array_equal(param_value.data(), values.data())

    def test_param_matrix(self):
        param = sr.Parameter("matrix", sr.StateType.PARAMETER_MATRIX)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "matrix")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_MATRIX)
        values = np.random.rand(3, 2)
        param.set_value(values)
        assert_array_equal(param.get_value(), values)

    def test_param_vector(self):
        param = sr.Parameter("vector", sr.StateType.PARAMETER_VECTOR)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "vector")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_VECTOR)
        values = np.random.rand(3)
        param.set_value(values)
        assert_array_equal(param.get_value(), values)


if __name__ == '__main__':
    unittest.main()
