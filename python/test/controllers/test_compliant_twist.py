import unittest

import numpy as np
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE


class TestCompliantTwistController(unittest.TestCase):
    ctrl = None
    command_twist = None
    feedback_twist = None

    @classmethod
    def setUpClass(cls):
        cls.ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        cls.command_twist = sr.CartesianTwist().Zero("test")
        cls.feedback_twist = sr.CartesianTwist().Zero("test")

    @classmethod
    def set_gains(cls, lpd, lod, ast, ad):
        cls.ctrl.set_parameter_value("linear_principle_damping", lpd, sr.StateType.PARAMETER_DOUBLE)
        cls.ctrl.set_parameter_value("linear_orthogonal_damping", lod, sr.StateType.PARAMETER_DOUBLE)
        cls.ctrl.set_parameter_value("angular_stiffness", ast, sr.StateType.PARAMETER_DOUBLE)
        cls.ctrl.set_parameter_value("angular_damping", ad, sr.StateType.PARAMETER_DOUBLE)

    def test_cartesian_wrench(self):
        self.set_gains(100, 100, 5, 5)

        command = self.ctrl.compute_command(self.command_twist, self.feedback_twist)
        # FIXME this line sometimes fails with github actions
        # self.assertTrue(np.linalg.norm(command.get_wrench()) < 1e-5)

        self.command_twist.set_linear_velocity(np.random.rand(3, 1))
        command = self.ctrl.compute_command(self.command_twist, self.feedback_twist)
        self.assertTrue(np.linalg.norm(command.get_force()) > 0.0)
        self.assertTrue(np.linalg.norm(command.get_torque()) < 1e-5)

        self.command_twist.set_linear_velocity([0, 0, 0])
        self.command_twist.set_angular_velocity(np.random.rand(3, 1))

        command = self.ctrl.compute_command(self.command_twist, self.feedback_twist)
        self.assertTrue(np.linalg.norm(command.get_force()) < 1e-5)
        self.assertTrue(np.linalg.norm(command.get_torque()) > 0)

    def test_get_and_set_parameters(self):
        self.set_gains(1, 2, 3, 4)
        parameter_map = self.ctrl.get_parameters()
        self.assertEqual(len(parameter_map), 4)
        for name, param in parameter_map.items():
            if param.get_name() == "linear_principle_damping":
                self.assertTrue(abs(param.get_value() - 1) < 1e-5)
                self.assertTrue(abs(self.ctrl.get_parameter_value("linear_principle_damping") - 1) < 1e-5)
                self.ctrl.set_parameter_value("linear_principle_damping", 11, sr.StateType.PARAMETER_DOUBLE)
                self.assertTrue(abs(self.ctrl.get_parameter_value("linear_principle_damping") - 11) < 1e-5)
            if param.get_name() == "linear_orthogonal_damping":
                self.assertTrue(abs(param.get_value() - 2) < 1e-5)
                self.assertTrue(abs(self.ctrl.get_parameter_value("linear_orthogonal_damping") - 2) < 1e-5)
                self.ctrl.set_parameter_value("linear_orthogonal_damping", 12, sr.StateType.PARAMETER_DOUBLE)
                self.assertTrue(abs(self.ctrl.get_parameter_value("linear_orthogonal_damping") - 12) < 1e-5)
            if param.get_name() == "angular_stiffness":
                self.assertTrue(abs(param.get_value() - 3) < 1e-5)
                self.assertTrue(abs(self.ctrl.get_parameter_value("angular_stiffness") - 3) < 1e-5)
                self.ctrl.set_parameter_value("angular_stiffness", 13, sr.StateType.PARAMETER_DOUBLE)
                self.assertTrue(abs(self.ctrl.get_parameter_value("angular_stiffness") - 13) < 1e-5)
            if param.get_name() == "angular_damping":
                self.assertTrue(abs(param.get_value() - 4) < 1e-5)
                self.assertTrue(abs(self.ctrl.get_parameter_value("angular_damping") - 4) < 1e-5)
                self.ctrl.set_parameter_value("angular_damping", 14, sr.StateType.PARAMETER_DOUBLE)
                self.assertTrue(abs(self.ctrl.get_parameter_value("angular_damping") - 14) < 1e-5)


if __name__ == '__main__':
    unittest.main()
