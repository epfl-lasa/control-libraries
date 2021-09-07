import unittest
from state_representation import State, CartesianState
import state_representation


class TestClproto(unittest.TestCase):
    def test_encode(self):
        state = CartesianState.Random("A")
        # state = State(state_representation.StateType.STATE, "A", True)
        print(state)
        print(state_representation.encoder(state))
        msg = state_representation.encode(state)
        print(msg)
        print(state_representation.is_valid(msg))
        state_type = state_representation.check_message_type(msg)
        print(state_type)
        result = CartesianState()
        print(state_representation.decode(msg, result))
        print(result)
        # print(state_representation.decode(msg))


if __name__ == '__main__':
    unittest.main()