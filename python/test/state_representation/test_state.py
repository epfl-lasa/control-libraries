import time
import unittest
import datetime
import copy

from state_representation import State, StateType

STATE_METHOD_EXPECTS = [
    'get_type',
    'is_empty',
    'set_empty',
    'set_filled',
    'get_timestamp',
    'set_timestamp',
    'reset_timestamp',
    'get_name',
    'set_name',
    'is_deprecated',
    'is_compatible',
    'initialize'
]


class TestState(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(State) if callable(getattr(State, m))]
        for expected in STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        empty1 = State()
        self.assertEqual(empty1.get_type(), StateType.STATE)
        self.assertEqual(empty1.get_name(), "")
        self.assertTrue(empty1.is_empty())

        empty2 = State(StateType.JOINT_STATE)
        self.assertEqual(empty2.get_type(), StateType.JOINT_STATE)
        self.assertEqual(empty2.get_name(), "")
        self.assertTrue(empty2.is_empty())

        empty3 = State(StateType.CARTESIAN_STATE, "test", True)
        self.assertEqual(empty3.get_type(), StateType.CARTESIAN_STATE)
        self.assertEqual(empty3.get_name(), "test")
        self.assertTrue(empty3.is_empty())
        empty3.set_filled()
        self.assertFalse(empty3.is_empty())

        state = State(empty3)
        self.assertEqual(state.get_type(), StateType.CARTESIAN_STATE)
        self.assertEqual(state.get_name(), "test")
        self.assertFalse(state.is_empty())
        state.set_empty()
        self.assertTrue(state.is_empty())

    def test_compatibility(self):
        state1 = State()
        state1.set_name("test")
        self.assertEqual(state1.get_name(), "test")

        state2 = State(StateType.STATE, "test", False)
        self.assertTrue(state1.is_compatible(state2))
        state2.set_name("world")
        self.assertFalse(state1.is_compatible(state2))

        state2.initialize()
        self.assertTrue(state2.is_empty())

    def test_timestamp(self):
        state = State(StateType.STATE, "test", False)
        time.sleep(0.2)
        self.assertTrue(state.is_deprecated(0.1))
        state.reset_timestamp()
        self.assertFalse(state.is_deprecated(0.1))
        time.sleep(0.2)
        self.assertTrue(state.is_deprecated(0.1))
        state.set_timestamp(datetime.datetime.now().timestamp())
        self.assertFalse(state.is_deprecated(0.1))

    def test_copy(self):
        state = State(StateType.STATE, "test", False)
        state2 = copy.copy(state)
        state3 = copy.deepcopy(state)

if __name__ == '__main__':
    unittest.main()
