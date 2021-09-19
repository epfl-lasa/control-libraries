import unittest

import numpy as np
from numpy.testing import assert_array_equal, assert_array_almost_equal
from state_representation import CartesianState, StateType, CartesianStateVariable

from .test_spatial_state import SPATIAL_STATE_METHOD_EXPECTS
from ..test_state import STATE_METHOD_EXPECTS

CARTESIAN_STATE_METHOD_EXPECTS = [
    'Identity',
    'Random',
    'array',
    'clamp_state_variable',
    'copy',
    'data',
    'set_data',
    'dist',
    'get_accelerations',
    'get_angular_acceleration',
    'get_angular_velocity',
    'get_force',
    'get_linear_acceleration',
    'get_linear_velocity',
    'get_name',
    'get_orientation',
    'get_pose',
    'get_position',
    'get_reference_frame',
    'get_timestamp',
    'get_torque',
    'get_transformation_matrix',
    'get_twist',
    'get_type',
    'get_wrench',
    'initialize',
    'inverse',
    'is_compatible',
    'is_deprecated',
    'is_empty',
    'normalize',
    'normalized',
    'norms',
    'reset_timestamp',
    'set_accelerations',
    'set_angular_acceleration',
    'set_angular_velocity',
    'set_empty',
    'set_filled',
    'set_force',
    'set_linear_acceleration',
    'set_linear_velocity',
    'set_name',
    'set_orientation',
    'set_pose',
    'set_position',
    'set_reference_frame',
    'set_torque',
    'set_twist',
    'set_wrench',
    'set_zero',
    'to_list'
]


class TestCartesianState(unittest.TestCase):
    def assert_name_empty_frame_equal(self, state, name, empty, reference_frame):
        self.assertEqual(state.get_name(), name)
        self.assertEqual(state.is_empty(), empty)
        self.assertEqual(state.get_reference_frame(), reference_frame)

    def assert_name_frame_data_equal(self, state1, state2):
        self.assertEqual(state1.get_name(), state2.get_name())
        self.assertEqual(state1.get_reference_frame(), state2.get_reference_frame())
        assert_array_almost_equal(state1.data(), state2.data())

    def clamping_helper(self, state, getter, setter, variable, dim):
        if dim == 3:
            data = [-2.0, 1.0, 5.0]
        elif dim == 6:
            data = [-2.0, 1.0, 5.0, 1.0, -3.0, 2.4]
        else:
            raise ValueError("Provide 3 or 6 as dimension")

        setter(data)
        state.clamp_state_variable(10.0, variable)
        self.assertEqual(type(state), CartesianState)
        [self.assertAlmostEqual(getter()[i], data[i]) for i in range(dim)]
        state.clamp_state_variable(3.0, variable)
        self.assertAlmostEqual(np.linalg.norm(getter()), 3.0)
        state.clamp_state_variable(10.0, variable, 0.5)
        self.assertAlmostEqual(np.linalg.norm(getter()), 0.0)

    def test_callable_methods(self):
        methods = [m for m in dir(CartesianState) if callable(getattr(CartesianState, m))]
        for expected in STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in SPATIAL_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in CARTESIAN_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)
        for expected in CARTESIAN_STATE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        empty1 = CartesianState()
        self.assertEqual(type(empty1), CartesianState)
        self.assertEqual(empty1.get_type(), StateType.CARTESIANSTATE)
        self.assert_name_empty_frame_equal(empty1, "", True, "world")
        self.assertAlmostEqual(np.linalg.norm(empty1.data()), 1)

        empty2 = CartesianState("test")
        self.assertEqual(type(empty1), CartesianState)
        self.assertEqual(empty2.get_type(), StateType.CARTESIANSTATE)
        self.assert_name_empty_frame_equal(empty2, "test", True, "world")
        self.assertAlmostEqual(np.linalg.norm(empty2.data()), 1)

        empty3 = CartesianState("test", "reference")
        self.assertEqual(type(empty1), CartesianState)
        self.assertEqual(empty3.get_type(), StateType.CARTESIANSTATE)
        self.assert_name_empty_frame_equal(empty3, "test", True, "reference")
        self.assertAlmostEqual(np.linalg.norm(empty3.data()), 1)

    def test_identity_initialization(self):
        identity = CartesianState().Identity("test")
        self.assertFalse(identity.is_empty())
        self.assertAlmostEqual(np.linalg.norm(identity.get_position()), 0)
        self.assertAlmostEqual(np.linalg.norm(identity.get_orientation()), 1)
        self.assertAlmostEqual(identity.get_orientation()[0], 1)
        self.assertAlmostEqual(np.linalg.norm(identity.get_twist()), 0)
        self.assertAlmostEqual(np.linalg.norm(identity.get_accelerations()), 0)
        self.assertAlmostEqual(np.linalg.norm(identity.get_wrench()), 0)

    def test_random_initialization(self):
        random = CartesianState().Random("test")
        self.assertFalse(random.is_empty())
        self.assertTrue(np.linalg.norm(random.get_position()) > 0)
        self.assertAlmostEqual(np.linalg.norm(random.get_orientation()), 1)
        [self.assertTrue(random.get_orientation()[i] != 0) for i in range(4)]
        self.assertTrue(np.linalg.norm(random.get_twist()) > 0)
        self.assertTrue(np.linalg.norm(random.get_accelerations()) > 0)
        self.assertTrue(np.linalg.norm(random.get_wrench()) > 0)

    def test_copy_constructor(self):
        random = CartesianState().Random("test")
        copy1 = random
        self.assert_name_frame_data_equal(random, copy1)

        copy2 = random.copy()
        self.assert_name_frame_data_equal(random, copy2)

        copy3 = CartesianState(random)
        self.assert_name_frame_data_equal(random, copy3)

        empty = CartesianState()
        copy4 = empty
        self.assertTrue(copy4.is_empty())
        copy5 = CartesianState(empty)
        self.assertTrue(copy5.is_empty())
        copy6 = empty.copy()
        self.assertTrue(copy6.is_empty())

    def test_get_set_fields(self):
        cs = CartesianState("test")

        # name
        cs.set_name("robot")
        self.assertEqual(cs.get_name(), "robot")
        self.assertEqual(cs.get_reference_frame(), "world")
        cs.set_reference_frame("base")
        self.assertEqual(cs.get_reference_frame(), "base")

        # position
        position = [1., 2., 3.]
        cs.set_position(position)
        [self.assertAlmostEqual(cs.get_position()[i], position[i]) for i in range(3)]
        cs.set_position(1.1, 2.2, 3.3)
        assert_array_equal(np.array([1.1, 2.2, 3.3]), cs.get_position())

        # orientation
        orientation_vec = np.random.rand(4)
        orientation_vec = orientation_vec / np.linalg.norm(orientation_vec)
        cs.set_orientation(orientation_vec)
        [self.assertAlmostEqual(cs.get_orientation()[i], orientation_vec[i]) for i in range(4)]
        # TODO what is an Eigen::Quaternion in Python ?
        with self.assertRaises(RuntimeError):
            cs.set_orientation(position)

        matrix = cs.get_transformation_matrix()
        trans = matrix[:3, 3]
        # rot = matrix[:3, :3]
        bottom = matrix[3, :]
        assert_array_almost_equal(trans, cs.get_position())
        # TODO rotation matrix from quaternion
        assert_array_equal(bottom, np.array([0, 0, 0, 1]))

        # pose
        position = [4.4, 5.5, 6.6]
        orientation_vec = np.random.rand(4)
        orientation_vec = orientation_vec / np.linalg.norm(orientation_vec)
        cs.set_pose(np.hstack((position, orientation_vec)))
        assert_array_almost_equal(np.hstack((position, orientation_vec)), cs.get_pose())
        with self.assertRaises(RuntimeError):
            cs.set_pose(position)

        # twist
        linear_velocity = np.random.rand(3)
        cs.set_linear_velocity(linear_velocity)
        assert_array_almost_equal(cs.get_linear_velocity(), linear_velocity)
        angular_velocity = np.random.rand(3)
        cs.set_angular_velocity(angular_velocity)
        assert_array_almost_equal(cs.get_angular_velocity(), angular_velocity)
        twist = np.random.rand(6)
        cs.set_twist(twist)
        assert_array_almost_equal(cs.get_twist(), twist)

        # acceleration
        linear_acceleration = np.random.rand(3)
        cs.set_linear_acceleration(linear_acceleration)
        assert_array_almost_equal(cs.get_linear_acceleration(), linear_acceleration)
        angular_acceleration = np.random.rand(3)
        cs.set_angular_acceleration(angular_acceleration)
        assert_array_almost_equal(cs.get_angular_acceleration(), angular_acceleration)
        accelerations = np.random.rand(6)
        cs.set_accelerations(accelerations)
        assert_array_almost_equal(cs.get_accelerations(), accelerations)

        # wrench
        force = np.random.rand(3)
        cs.set_force(force)
        assert_array_almost_equal(cs.get_force(), force)
        torque = np.random.rand(3)
        cs.set_torque(torque)
        assert_array_almost_equal(cs.get_torque(), torque)
        wrench = np.random.rand(6)
        cs.set_wrench(wrench)
        assert_array_almost_equal(cs.get_wrench(), wrench)

        cs.set_zero()
        self.assertAlmostEqual(np.linalg.norm(cs.data()), 1)
        self.assertFalse(cs.is_empty())
        cs.set_empty()
        self.assertTrue(cs.is_empty())

    def test_compatibility(self):
        cs1 = CartesianState("test")
        cs2 = CartesianState("robot")
        cs3 = CartesianState("robot", "test")
        cs4 = CartesianState("test", "robot")

        self.assertFalse(cs1.is_compatible(cs2))
        self.assertFalse(cs1.is_compatible(cs3))
        self.assertFalse(cs1.is_compatible(cs4))

    def test_set_zero(self):
        random1 = CartesianState().Random("test")
        random1.initialize()
        self.assertAlmostEqual(np.linalg.norm(random1.data()), 1)

        random2 = CartesianState().Random("test")
        random2.set_zero()
        self.assertAlmostEqual(np.linalg.norm(random1.data()), 1)

    def test_get_set_data(self):
        cs1 = CartesianState().Identity("test")
        cs2 = CartesianState().Random("test")
        concatenated_state = np.hstack((cs1.get_pose(), cs1.get_twist(), cs1.get_accelerations(), cs1.get_wrench()))
        assert_array_almost_equal(cs1.data(), concatenated_state)
        assert_array_almost_equal(cs1.array(), concatenated_state)

        cs1.set_data(cs2.data())
        assert_array_almost_equal(cs1.data(), cs2.data())

        cs2 = CartesianState.Random("test")
        state_vec = cs2.to_list()
        cs1.set_data(state_vec)
        [self.assertAlmostEqual(cs1.data()[i], state_vec[i]) for i in range(len(state_vec))]

        with self.assertRaises(RuntimeError):
            cs1.set_data(np.array([0, 0]))

    def test_clamping(self):
        state = CartesianState().Identity("test")
        with self.assertRaises(RuntimeError):
            state.clamp_state_variable(1, CartesianStateVariable.ORIENTATION)
        with self.assertRaises(RuntimeError):
            state.clamp_state_variable(1, CartesianStateVariable.POSE)
        with self.assertRaises(RuntimeError):
            state.clamp_state_variable(1, CartesianStateVariable.ALL)

        self.clamping_helper(state, state.get_position, state.set_position, CartesianStateVariable.POSITION, 3)
        self.clamping_helper(state, state.get_linear_velocity, state.set_linear_velocity,
                             CartesianStateVariable.LINEAR_VELOCITY, 3)
        self.clamping_helper(state, state.get_angular_velocity, state.set_angular_velocity,
                             CartesianStateVariable.ANGULAR_VELOCITY, 3)
        self.clamping_helper(state, state.get_twist, state.set_twist, CartesianStateVariable.TWIST, 6)
        self.clamping_helper(state, state.get_linear_acceleration, state.set_linear_acceleration,
                             CartesianStateVariable.LINEAR_ACCELERATION, 3)
        self.clamping_helper(state, state.get_angular_acceleration, state.set_angular_acceleration,
                             CartesianStateVariable.ANGULAR_ACCELERATION, 3)
        self.clamping_helper(state, state.get_accelerations, state.set_accelerations,
                             CartesianStateVariable.ACCELERATIONS, 6)
        self.clamping_helper(state, state.get_force, state.set_force, CartesianStateVariable.FORCE, 3)
        self.clamping_helper(state, state.get_torque, state.set_torque, CartesianStateVariable.TORQUE, 3)
        self.clamping_helper(state, state.get_wrench, state.set_wrench, CartesianStateVariable.WRENCH, 6)

    def test_norms(self):
        cs = CartesianState().Random("test")
        norms = cs.norms()
        self.assertEqual(len(norms), 8)
        self.assertAlmostEqual(norms[0], np.linalg.norm(cs.get_position()))
        self.assertAlmostEqual(norms[1], np.linalg.norm(cs.get_orientation()))
        self.assertAlmostEqual(norms[2], np.linalg.norm(cs.get_linear_velocity()))
        self.assertAlmostEqual(norms[3], np.linalg.norm(cs.get_angular_velocity()))
        self.assertAlmostEqual(norms[4], np.linalg.norm(cs.get_linear_acceleration()))
        self.assertAlmostEqual(norms[5], np.linalg.norm(cs.get_angular_acceleration()))
        self.assertAlmostEqual(norms[6], np.linalg.norm(cs.get_force()))
        self.assertAlmostEqual(norms[7], np.linalg.norm(cs.get_torque()))

    def test_normalize(self):
        cs = CartesianState().Random("test")
        normalized = cs.normalized()
        self.assertEqual(type(normalized), CartesianState)
        norms1 = normalized.norms()
        [self.assertAlmostEqual(n, 1) for n in norms1]

        cs.normalize()
        norms2 = cs.norms()
        [self.assertAlmostEqual(n, 1) for n in norms2]

    def test_distance(self):
        empty = CartesianState()
        cs1 = CartesianState().Random("test")
        cs2 = CartesianState().Random("test", "robot")

        with self.assertRaises(RuntimeError):
            empty.dist(cs1)
        with self.assertRaises(RuntimeError):
            cs1.dist(empty)
        with self.assertRaises(RuntimeError):
            cs1.dist(cs2)

        data1 = np.random.rand(25)
        cs1.set_data(data1)
        cs3 = CartesianState("test")
        data3 = np.random.rand(25)
        cs3.set_data(data3)

        pos_dist = np.linalg.norm(data1[:3] - data3[:3])
        # TODO orientation distance
        orient_dist = cs1.dist(cs3, CartesianStateVariable.ORIENTATION)
        lin_vel_dist = np.linalg.norm(data1[7:10] - data3[7:10])
        ang_vel_dist = np.linalg.norm(data1[10:13] - data3[10:13])
        lin_acc_dist = np.linalg.norm(data1[13:16] - data3[13:16])
        ang_acc_dist = np.linalg.norm(data1[16:19] - data3[16:19])
        force_dist = np.linalg.norm(data1[19:22] - data3[19:22])
        torque_dist = np.linalg.norm(data1[22:] - data3[22:])

        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.POSITION), pos_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.ORIENTATION), orient_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.POSE), pos_dist + orient_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.TWIST), lin_vel_dist + ang_vel_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.ACCELERATIONS), lin_acc_dist + ang_acc_dist)
        self.assertAlmostEqual(cs1.dist(cs3, CartesianStateVariable.WRENCH), force_dist + torque_dist)
        self.assertAlmostEqual(cs1.dist(cs3), cs3.dist(cs1, CartesianStateVariable.ALL))

    def test_addition(self):
        cs1 = CartesianState().Random("test")
        cs2 = CartesianState().Random("test")
        cs3 = CartesianState().Random("test", "reference")

        with self.assertRaises(RuntimeError):
            cs1 + cs3

        csum = cs1 + cs2
        self.assertEqual(type(csum), CartesianState)
        assert_array_almost_equal(csum.get_position(), cs1.get_position() + cs2.get_position())
        # TODO orientation sum
        assert_array_almost_equal(csum.get_twist(), cs1.get_twist() + cs2.get_twist())
        assert_array_almost_equal(csum.get_accelerations(), cs1.get_accelerations() + cs2.get_accelerations())
        assert_array_almost_equal(csum.get_wrench(), cs1.get_wrench() + cs2.get_wrench())

        cs1 += cs2
        self.assertEqual(type(cs1), CartesianState)
        assert_array_almost_equal(cs1.data(), csum.data())

    def test_subtraction(self):
        cs1 = CartesianState().Random("test")
        cs2 = CartesianState().Random("test")
        cs3 = CartesianState().Random("test", "reference")

        with self.assertRaises(RuntimeError):
            cs1 - cs3

        cdiff = cs1 - cs2
        self.assertEqual(type(cdiff), CartesianState)
        assert_array_almost_equal(cdiff.get_position(), cs1.get_position() - cs2.get_position())
        # TODO orientation diff
        assert_array_almost_equal(cdiff.get_twist(), cs1.get_twist() - cs2.get_twist())
        assert_array_almost_equal(cdiff.get_accelerations(), cs1.get_accelerations() - cs2.get_accelerations())
        assert_array_almost_equal(cdiff.get_wrench(), cs1.get_wrench() - cs2.get_wrench())

        cs1 -= cs2
        self.assertEqual(type(cs1), CartesianState)
        assert_array_almost_equal(cs1.data(), cdiff.data())

    def test_scalar_multiplication(self):
        scalar = 2.0
        cs = CartesianState().Random("test")
        cscaled = scalar * cs
        self.assertEqual(type(cscaled), CartesianState)
        assert_array_almost_equal(cscaled.get_position(), scalar * cs.get_position())
        # TODO orientation mult
        assert_array_almost_equal(cscaled.get_twist(), scalar * cs.get_twist())
        assert_array_almost_equal(cscaled.get_accelerations(), scalar * cs.get_accelerations())
        assert_array_almost_equal(cscaled.get_wrench(), scalar * cs.get_wrench())

        cs *= scalar
        self.assertEqual(type(cs), CartesianState)
        assert_array_almost_equal(cs.data(), cscaled.data())

        empty = CartesianState()
        with self.assertRaises(RuntimeError):
            scalar * empty

    def test_scalar_division(self):
        scalar = 2.0
        cs = CartesianState().Random("test")
        cscaled = cs / scalar
        self.assertEqual(type(cscaled), CartesianState)
        assert_array_almost_equal(cscaled.get_position(), cs.get_position() / scalar)
        # TODO orientation diff
        assert_array_almost_equal(cscaled.get_twist(), cs.get_twist() / scalar)
        assert_array_almost_equal(cscaled.get_accelerations(), cs.get_accelerations() / scalar)
        assert_array_almost_equal(cscaled.get_wrench(), cs.get_wrench() / scalar)

        cs /= scalar
        self.assertEqual(type(cs), CartesianState)
        assert_array_almost_equal(cs.data(), cscaled.data())

        with self.assertRaises(RuntimeError):
            cs / 0.0

        empty = CartesianState()
        with self.assertRaises(RuntimeError):
            empty / scalar


if __name__ == '__main__':
    unittest.main()
