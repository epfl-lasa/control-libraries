import state_representation as sr
import unittest
from datetime import timedelta
from dynamical_systems import JointPointAttractorDS


class TestJointPointAttractor(unittest.TestCase):
    def test_empty_constructor(self):
        ds = JointPointAttractorDS()
        attractor = sr.JointPositions.Zero("robot", 3)

        self.assertTrue(ds.get_parameter_value("attractor").is_empty())
        self.assertTrue(ds.get_base_frame().is_empty())
        ds.set_parameter(sr.Parameter("attractor", attractor, sr.StateType.PARAMETER_JOINTPOSITIONS))
        self.assertFalse(ds.get_parameter_value("attractor").is_empty())

    def test_convergence(self):
        ds = JointPointAttractorDS()
        attractor = sr.JointPositions.Random("robot", 3)
        ds.set_parameter(sr.Parameter("attractor", attractor, sr.StateType.PARAMETER_JOINTPOSITIONS))

        current_state = sr.JointPositions.Random("robot", 3)
        current_state.set_data(10 * current_state.data())
        for i in range(100):
            velocities = sr.JointVelocities(ds.evaluate(current_state))
            current_state += timedelta(milliseconds=100) * velocities

        self.assertTrue(current_state.dist(attractor, sr.JointStateVariable.POSITIONS) < 1e-3)


if __name__ == '__main__':
    unittest.main()
