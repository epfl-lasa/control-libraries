import unittest
import state_representation as sr

import numpy as np
from numpy.testing import assert_array_equal


class TestParameters(unittest.TestCase):

    def cartesian_equal(self, cartesian1, cartesian2):
        self.assertTrue(cartesian1.get_name(), cartesian2.get_name())
        self.assertTrue(cartesian1.get_reference_frame(), cartesian2.get_reference_frame())
        assert_array_equal(cartesian1.data(), cartesian2.data())

    def joint_equal(self, joint1, joint2):
        self.assertTrue(joint1.get_name(), joint2.get_name())
        self.assertTrue(joint1.get_size(), joint2.get_size())
        assert_array_equal(joint1.data(), joint2.data())

    def test_param_invalid(self):
        param = sr.Parameter("test", sr.StateType.CARTESIANSTATE)
        self.assertRaises(ValueError, param.set_value, 1)

    def test_param_copy(self):
        param = sr.Parameter("test", 1, sr.StateType.PARAMETER_INT)
        self.assertEqual(param.get_name(), "test")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_INT)
        self.assertEqual(param.get_value(), 1)
        param_copy = sr.Parameter(param)
        self.assertEqual(param_copy.get_name(), "test")
        self.assertEqual(param_copy.get_type(), sr.StateType.PARAMETER_INT)
        self.assertEqual(param_copy.get_value(), 1)

    def test_param_int(self):
        param = sr.Parameter("int", sr.StateType.PARAMETER_INT)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "int")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_INT)
        param.set_value(1)
        self.assertEqual(param.get_value(), 1)
        param1 = sr.Parameter("int", 1, sr.StateType.PARAMETER_INT)
        self.assertEqual(param1.get_value(), 1)

    def test_param_int_array(self):
        param = sr.Parameter("int_array", sr.StateType.PARAMETER_INT_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "int_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_INT_ARRAY)
        values = [2, 3, 4]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("int_array", values, sr.StateType.PARAMETER_INT_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_double(self):
        param = sr.Parameter("double", sr.StateType.PARAMETER_DOUBLE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "double")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_DOUBLE)
        param.set_value(1.5)
        self.assertEqual(param.get_value(), 1.5)
        param1 = sr.Parameter("double", 1.5, sr.StateType.PARAMETER_DOUBLE)
        self.assertEqual(param1.get_value(), 1.5)

    def test_param_double_array(self):
        param = sr.Parameter("double_array", sr.StateType.PARAMETER_DOUBLE_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "double_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_DOUBLE_ARRAY)
        values = [2.2, 3.3, 4.4]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("double_array", values, sr.StateType.PARAMETER_DOUBLE_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_bool(self):
        param = sr.Parameter("bool", sr.StateType.PARAMETER_BOOL)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "bool")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_BOOL)
        param.set_value(False)
        self.assertEqual(param.get_value(), False)
        param1 = sr.Parameter("bool", False, sr.StateType.PARAMETER_BOOL)
        self.assertEqual(param1.get_value(), False)

    def test_param_bool_array(self):
        param = sr.Parameter("bool_array", sr.StateType.PARAMETER_BOOL_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "bool_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_BOOL_ARRAY)
        values = [True, False, False]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("bool_array", values, sr.StateType.PARAMETER_BOOL_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_string(self):
        param = sr.Parameter("string", sr.StateType.PARAMETER_STRING)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "string")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_STRING)
        param.set_value("parameter")
        self.assertEqual(param.get_value(), "parameter")
        param1 = sr.Parameter("string", "parameter", sr.StateType.PARAMETER_STRING)
        self.assertEqual(param1.get_value(), "parameter")

    def test_param_string_array(self):
        param = sr.Parameter("string_array", sr.StateType.PARAMETER_STRING_ARRAY)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "string_array")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_STRING_ARRAY)
        values = ["test", "parameter", "bindings"]
        param.set_value(values)
        [self.assertEqual(param.get_value()[i], values[i]) for i in range(len(values))]
        param1 = sr.Parameter("string_array", values, sr.StateType.PARAMETER_STRING_ARRAY)
        [self.assertEqual(param1.get_value()[i], values[i]) for i in range(len(values))]

    def test_param_cartesian_state(self):
        param = sr.Parameter("cartesian_state", sr.StateType.PARAMETER_CARTESIANSTATE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "cartesian_state")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_CARTESIANSTATE)
        values = sr.CartesianState.Random("test")
        param.set_value(values)
        self.cartesian_equal(param.get_value(), values)
        param1 = sr.Parameter("cartesian_state", values, sr.StateType.PARAMETER_CARTESIANSTATE)
        self.cartesian_equal(param1.get_value(), values)

    def test_param_cartesian_pose(self):
        param = sr.Parameter("cartesian_pose", sr.StateType.PARAMETER_CARTESIANPOSE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "cartesian_pose")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_CARTESIANPOSE)
        values = sr.CartesianPose.Random("test")
        param.set_value(values)
        self.cartesian_equal(param.get_value(), values)
        param1 = sr.Parameter("cartesian_pose", values, sr.StateType.PARAMETER_CARTESIANPOSE)
        self.cartesian_equal(param1.get_value(), values)

    def test_param_joint_state(self):
        param = sr.Parameter("joint_state", sr.StateType.PARAMETER_JOINTSTATE)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "joint_state")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_JOINTSTATE)
        values = sr.JointState.Random("test", 3)
        param.set_value(values)
        self.joint_equal(param.get_value(), values)
        param1 = sr.Parameter("joint_state", values, sr.StateType.PARAMETER_JOINTSTATE)
        self.joint_equal(param1.get_value(), values)

    def test_param_cartesian_pose(self):
        param = sr.Parameter("joint_positions", sr.StateType.PARAMETER_JOINTPOSITIONS)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "joint_positions")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_JOINTPOSITIONS)
        values = sr.JointPositions.Random("test", 3)
        param.set_value(values)
        self.joint_equal(param.get_value(), values)
        param1 = sr.Parameter("joint_positions", values, sr.StateType.PARAMETER_JOINTPOSITIONS)
        self.joint_equal(param1.get_value(), values)

    def test_param_matrix(self):
        param = sr.Parameter("matrix", sr.StateType.PARAMETER_MATRIX)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "matrix")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_MATRIX)
        values = np.random.rand(3, 2)
        param.set_value(values)
        assert_array_equal(param.get_value(), values)
        param1 = sr.Parameter("matrix", values, sr.StateType.PARAMETER_MATRIX)
        assert_array_equal(param1.get_value(), values)

    def test_param_vector(self):
        param = sr.Parameter("vector", sr.StateType.PARAMETER_VECTOR)
        self.assertTrue(param.is_empty())
        self.assertEqual(param.get_name(), "vector")
        self.assertEqual(param.get_type(), sr.StateType.PARAMETER_VECTOR)
        values = np.random.rand(3)
        param.set_value(values)
        assert_array_equal(param.get_value(), values)
        param1 = sr.Parameter("vector", values, sr.StateType.PARAMETER_VECTOR)
        assert_array_equal(param1.get_value(), values)


if __name__ == '__main__':
    unittest.main()
