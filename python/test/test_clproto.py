import unittest
from state_representation import State, CartesianState
import clproto


class TestClproto(unittest.TestCase):
    def test_encode(self):
        state = CartesianState.Random("A")
        print(state)
        # msg = clproto.encode(state)
        # print(msg)


if __name__ == '__main__':
    unittest.main()