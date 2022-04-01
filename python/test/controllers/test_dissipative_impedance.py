import unittest

import numpy as np
import state_representation as sr
from controllers import create_cartesian_controller, create_joint_controller, CONTROLLER_TYPE


class TestDissipativeImpedance(unittest.TestCase):

    def test_compute_command_with_colinear_velocity(self):
        tolerance = 1e-4
        ctrl = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE_LINEAR)

        eigenvalues = ctrl.get_parameter_value("damping_eigenvalues")
        eigenvalues[0] = 10
        ctrl.set_parameter_value("damping_eigenvalues", eigenvalues, sr.StateType.PARAMETER_VECTOR)

        desired_twist = sr.CartesianTwist("test", [1, 0, 0])
        feedback_twist = sr.CartesianTwist("test", [1, 1, 0])

        command = ctrl.compute_command(desired_twist, sr.CartesianTwist.Zero("test"))
        self.assertTrue(abs(command.get_force()[0] - 10) < tolerance)
        self.assertTrue(abs(command.get_force()[1]) < tolerance)
        self.assertTrue(abs(command.get_force()[2]) < tolerance)
        command = ctrl.compute_command(desired_twist, feedback_twist)
        self.assertTrue(abs(command.get_force()[0]) < tolerance)
        self.assertTrue(abs(command.get_force()[1] - -1) < tolerance)
        self.assertTrue(abs(command.get_force()[2]) < tolerance)

    def test_compute_joint_command(self):
        ctrl = create_joint_controller(CONTROLLER_TYPE.DISSIPATIVE, 4)

        desired_velocities = sr.JointVelocities("test", [1, 0, 0, 0])
        feedback_velocities = sr.JointVelocities("test", [1, 1, 0, 0])

        command = ctrl.compute_command(desired_velocities, feedback_velocities)
        self.assertTrue(np.linalg.norm(command.get_torques()) > 0)

    def test_compute_task_to_joint_command(self):
        ctrl = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE)

        desired_twist = sr.CartesianTwist("test", [1, 0, 0])
        feedback_twist = sr.CartesianTwist("test", [1, 1, 0])
        jac = sr.Jacobian().Random("test_robot", 3, "test")

        command = ctrl.compute_command(desired_twist, feedback_twist, jac)
        self.assertTrue(np.linalg.norm(command.get_torques()) > 0)


if __name__ == '__main__':
    unittest.main()
