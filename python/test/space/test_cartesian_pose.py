import unittest
import copy

from state_representation import CartesianPose
import numpy as np

class TestCartesianPose(unittest.TestCase):
    def assert_np_array_equal(self, a, b):
        self.assertListEqual(list(a), list(b))

    def test_constructors(self):
        CartesianPose("A", "B")
        A = CartesianPose("A", np.array([1, 2, 3]), "B")
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])
        self.assert_np_array_equal(A.get_orientation(), [1, 0, 0, 0])

        A = CartesianPose("A", np.array([1, 2, 3, 4]), "B")
        self.assert_np_array_equal(A.get_position(), [0, 0, 0])
        self.assert_np_array_equal(A.get_orientation(), [1, 2, 3, 4] / np.linalg.norm([1, 2, 3, 4]))

        A = CartesianPose("A", np.array([1, 2, 3]), np.array([1, 2, 3, 4]), "B")
        self.assert_np_array_equal(A.get_position(), [1, 2, 3])
        self.assert_np_array_equal(A.get_orientation(), [1, 2, 3, 4] / np.linalg.norm([1, 2, 3, 4]))

    def test_copy(self):
        state = CartesianPose().Random("test")
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assertEqual(state.get_name(), state_copy.get_name())
            self.assertEqual(state.get_reference_frame(), state_copy.get_reference_frame())
            self.assert_np_array_equal(state.data(), state_copy.data())

    def test_inverse(self):
        pose = CartesianPose.Random("A", "B")
        inv_pose = pose.inverse()
        self.assertIsInstance(inv_pose, CartesianPose)

if __name__ == '__main__':
    unittest.main()