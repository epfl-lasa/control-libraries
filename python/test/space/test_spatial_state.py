import unittest
import copy

from state_representation import SpatialState, StateType
from test.test_state import STATE_METHOD_EXPECTS

SPATIAL_STATE_METHOD_EXPECTS = [
    'get_reference_frame',
    'set_reference_frame',
    'is_compatible'
]


class TestState(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(SpatialState) if callable(getattr(SpatialState, m))]
        for expected in STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in SPATIAL_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        state1 = SpatialState(StateType.JOINTSTATE)
        self.assertEqual(state1.get_type(), StateType.JOINTSTATE)
        self.assertEqual(state1.get_name(), "")
        self.assertEqual(state1.get_reference_frame(), "world")
        self.assertTrue(state1.is_empty())

        state2 = SpatialState(StateType.CARTESIANSTATE, "test", "robot", False)
        self.assertEqual(state2.get_type(), StateType.CARTESIANSTATE)
        self.assertEqual(state2.get_name(), "test")
        self.assertEqual(state2.get_reference_frame(), "robot")
        self.assertFalse(state2.is_empty())

        state3 = SpatialState(state2)
        self.assertEqual(state2.get_type(), state3.get_type())
        self.assertEqual(state2.get_name(), state3.get_name())
        self.assertEqual(state2.get_reference_frame(), state3.get_reference_frame())
        self.assertEqual(state2.is_empty(), state3.is_empty())

    def test_copy(self):
        state = SpatialState(StateType.CARTESIANSTATE, "test", "robot", False)
        for state_copy in [copy.copy(state), copy.deepcopy(state)]:
            self.assertEqual(state.get_type(), state_copy.get_type())
            self.assertEqual(state.get_name(), state_copy.get_name())
            self.assertEqual(state.get_reference_frame(), state_copy.get_reference_frame())
            self.assertEqual(state.is_empty(), state_copy.is_empty())

    def test_compatibility(self):
        state1 = SpatialState(StateType.CARTESIANSTATE, "test", "robot", True)
        state2 = SpatialState(StateType.STATE, "test")
        self.assertFalse(state1.is_compatible(state2))
        state2.set_reference_frame("robot")
        self.assertTrue(state1.is_compatible(state2))


if __name__ == '__main__':
    unittest.main()
