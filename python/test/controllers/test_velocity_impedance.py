import unittest

import numpy as np
import state_representation as sr
from controllers import create_cartesian_controller, create_joint_controller, CONTROLLER_TYPE


class TestVelocityImpedance(unittest.TestCase):

    def test_cartesian_impedance(self):
        ctrl = create_cartesian_controller(CONTROLLER_TYPE.VELOCITY_IMPEDANCE)

        desired_state = sr.CartesianState("test")
        desired_state.set_linear_velocity([1, 0, 0])
        feedback_state = sr.CartesianState("test")
        feedback_state.set_linear_velocity([0.5, 0, 0])

        command = ctrl.compute_command(desired_state, feedback_state)
        self.assertTrue(np.linalg.norm(command.data()) > 0)

        feedback_state.set_linear_velocity(desired_state.get_linear_velocity())
        command = ctrl.compute_command(desired_state, feedback_state)
        self.assertTrue(np.linalg.norm(command.data()) > 0)

    def test_joint_impedance(self):
        nb_joints = 3
        ctrl = create_joint_controller(CONTROLLER_TYPE.VELOCITY_IMPEDANCE, nb_joints)

        desired_state = sr.JointState("test", nb_joints)
        desired_state.set_velocities([1, 0, 0])
        feedback_state = sr.JointState("test", nb_joints)
        feedback_state.set_velocities([0.5, 0, 0])

        command = ctrl.compute_command(desired_state, feedback_state)
        self.assertTrue(np.linalg.norm(command.data()) > 0)

        feedback_state.set_velocities(desired_state.get_velocities())
        command = ctrl.compute_command(desired_state, feedback_state)
        self.assertTrue(np.linalg.norm(command.data()) > 0)

    def test_cartesian_to_joint_impedance(self):
        ctrl = create_cartesian_controller(CONTROLLER_TYPE.VELOCITY_IMPEDANCE)

        desired_state = sr.CartesianState("test")
        desired_state.set_linear_velocity([1, 0, 0])
        feedback_state = sr.CartesianState("test")
        feedback_state.set_linear_velocity([0.5, 0, 0])

        jac = sr.Jacobian().Random("test_robot", 3, "test")

        command = ctrl.compute_command(desired_state, feedback_state, jac)
        self.assertTrue(np.linalg.norm(command.data()) > 0)

        feedback_state.set_linear_velocity(desired_state.get_linear_velocity())
        command = ctrl.compute_command(desired_state, feedback_state, jac)
        self.assertTrue(np.linalg.norm(command.data()) > 0)


if __name__ == '__main__':
    unittest.main()
