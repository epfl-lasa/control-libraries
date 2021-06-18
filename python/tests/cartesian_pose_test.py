import unittest
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

if __name__ == '__main__':
    unittest.main()