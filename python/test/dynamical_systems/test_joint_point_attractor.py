import state_representation as sr
import unittest
from datetime import timedelta
from dynamical_systems import create_joint_ds, DYNAMICAL_SYSTEM_TYPE


class TestJointPointAttractor(unittest.TestCase):
    def test_constructor(self):
        ds = create_joint_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        attractor = sr.JointState.Zero("robot", 3)

        self.assertTrue(ds.get_parameter_value("attractor").is_empty())
        self.assertTrue(ds.get_base_frame().is_empty())
        ds.set_parameter(sr.Parameter("attractor", attractor, sr.ParameterType.STATE, sr.StateType.JOINT_STATE))
        self.assertFalse(ds.get_parameter_value("attractor").is_empty())

        parameters = [sr.Parameter("attractor", attractor, sr.ParameterType.STATE, sr.StateType.JOINT_STATE)]
        ds = create_joint_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR, parameters)
        self.assertFalse(ds.get_parameter_value("attractor").is_empty())

    def test_convergence(self):
        ds = create_joint_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        attractor = sr.JointPositions.Random("robot", 3)
        ds.set_parameter(sr.Parameter("attractor", attractor, sr.ParameterType.STATE, sr.StateType.JOINT_STATE))

        current_state = sr.JointPositions.Random("robot", 3)
        current_state.set_data(10 * current_state.data())
        for i in range(100):
            velocities = sr.JointVelocities(ds.evaluate(current_state))
            current_state += timedelta(milliseconds=100) * velocities

        self.assertTrue(current_state.dist(attractor, sr.JointStateVariable.POSITIONS) < 1e-3)


if __name__ == '__main__':
    unittest.main()
