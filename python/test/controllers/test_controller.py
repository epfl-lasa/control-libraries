import os
import unittest

import state_representation as sr
from controllers import ICartesianController, create_cartesian_controller, create_joint_controller, CONTROLLER_TYPE
from robot_model import Model

CONTROLLER_METHOD_EXPECTS = [
    'compute_command',
    'get_robot_model',
    'set_robot_model',
    'get_parameter',
    'get_parameters',
    'get_parameter_value',
    'get_parameter_list',
    'set_parameter',
    'set_parameters',
    'set_parameter_value'
]


class TestControllers(unittest.TestCase):

    def expected_parameters_test(self, controller, expected_params, dim=6):
        params = controller.get_parameters()
        [self.assertTrue(key in params.keys()) for key in expected_params]
        for param in expected_params:
            p = controller.get_parameter_value(param)
            self.assertTrue(p.size == dim * dim)

    def test_callable_methods(self):
        methods = [m for m in dir(ICartesianController) if callable(getattr(ICartesianController, m))]
        for expected in CONTROLLER_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_cartesian_controller(self):
        command_state = sr.CartesianState().Identity("command")
        feedback_state = sr.CartesianState().Identity("feedback")

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.NONE)
        self.assertTrue(ctrl is None)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping", "stiffness", "inertia"])
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.VELOCITY_IMPEDANCE)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping", "stiffness"])
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping"])
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE_LINEAR)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping"])
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE_ANGULAR)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping"])
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE_DECOUPLED)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping"])
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        self.assertFalse(ctrl is None)
        parameters = ctrl.get_parameters()
        expected_parameters = ["linear_principle_damping", "linear_orthogonal_damping", "angular_stiffness",
                               "angular_damping"]
        [self.assertTrue(key in parameters.keys()) for key in expected_parameters]
        ctrl.compute_command(command_state, feedback_state)
        self.assertRaises(RuntimeError, ctrl.compute_command, command_state, feedback_state,
                          sr.JointPositions().Zero("robot", 3))

    def test_joint_controller(self):
        dim = 3

        command_state = sr.JointState().Zero("robot", dim)
        feedback_state = sr.JointState().Zero("robot", dim)

        ctrl = create_joint_controller(CONTROLLER_TYPE.NONE, 3)
        self.assertEqual(ctrl, None)

        ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, dim)
        self.assertNotEqual(ctrl, None)
        self.expected_parameters_test(ctrl, ["damping", "stiffness", "inertia"], dim)
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_joint_controller(CONTROLLER_TYPE.VELOCITY_IMPEDANCE, dim)
        self.assertNotEqual(ctrl, None)
        self.expected_parameters_test(ctrl, ["damping", "stiffness"], dim)
        ctrl.compute_command(command_state, feedback_state)

        ctrl = create_joint_controller(CONTROLLER_TYPE.DISSIPATIVE, dim)
        self.assertFalse(ctrl is None)
        self.expected_parameters_test(ctrl, ["damping"], dim)

        self.assertRaises(RuntimeError, create_joint_controller, CONTROLLER_TYPE.DISSIPATIVE_LINEAR)
        self.assertRaises(RuntimeError, create_joint_controller, CONTROLLER_TYPE.DISSIPATIVE_ANGULAR)
        self.assertRaises(RuntimeError, create_joint_controller, CONTROLLER_TYPE.DISSIPATIVE_DECOUPLED)
        self.assertRaises(RuntimeError, create_joint_controller, CONTROLLER_TYPE.COMPLIANT_TWIST)

    def test_controller_with_params(self):
        param_list = [sr.Parameter("damping", 0.0, sr.StateType.PARAMETER_DOUBLE),
                      sr.Parameter("stiffness", 5.0, sr.StateType.PARAMETER_DOUBLE),
                      sr.Parameter("inertia", 10.0, sr.StateType.PARAMETER_DOUBLE)]

        ctrl = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE, param_list)
        self.assertFalse(ctrl is None)

        self.assertEqual(ctrl.get_parameter_value("damping").size, 6 * 6)
        self.assertEqual(ctrl.get_parameter_value("stiffness").size, 6 * 6)
        self.assertEqual(ctrl.get_parameter_value("inertia").size, 6 * 6)

        self.assertEqual(ctrl.get_parameter_value("damping").sum(), 0.0 * 6)
        self.assertEqual(ctrl.get_parameter_value("stiffness").sum(), 5.0 * 6)
        self.assertEqual(ctrl.get_parameter_value("inertia").sum(), 10.0 * 6)

    def test_controller_with_robot(self):
        robot_name = "robot"
        urdf_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "panda_arm.urdf")
        robot = Model(robot_name, urdf_path)

        command_state = sr.CartesianState().Identity(robot.get_frames()[-1], robot.get_base_frame())
        feedback_state = sr.CartesianState().Identity(robot.get_frames()[-1], robot.get_base_frame())
        joint_state = sr.JointPositions().Zero(robot_name, robot.get_number_of_joints())

        self.assertRaises(RuntimeError, create_cartesian_controller, CONTROLLER_TYPE.NONE, robot)
        self.assertRaises(RuntimeError, create_joint_controller, CONTROLLER_TYPE.NONE, robot)

        cart_ctrl = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE, robot)
        self.assertNotEqual(cart_ctrl, None)

        cart_ctrl.compute_command(command_state, feedback_state)
        cart_ctrl.compute_command(command_state, feedback_state, joint_state)
        cart_ctrl.compute_command(command_state, feedback_state, robot.compute_jacobian(joint_state))

        joint_ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, robot)
        self.assertNotEqual(joint_ctrl, None)
        joint_ctrl.compute_command(joint_state, joint_state)
        self.assertEqual(joint_ctrl.get_parameter_value("stiffness").size,
                         robot.get_number_of_joints() * robot.get_number_of_joints())

    def test_controller_with_robot_and_params(self):
        parameters = [sr.Parameter("damping", 5.0, sr.StateType.PARAMETER_DOUBLE)]
        robot = Model("robot", os.path.join(os.path.dirname(os.path.realpath(__file__)), "panda_arm.urdf"))

        self.assertRaises(RuntimeError, create_joint_controller, CONTROLLER_TYPE.NONE, parameters, robot)

        ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, parameters, robot)
        self.assertNotEqual(ctrl, None)
        self.assertEqual(ctrl.get_parameter_value("damping").size,
                         robot.get_number_of_joints() * robot.get_number_of_joints())

        self.assertEqual(ctrl.get_parameter_value("damping").sum(), 5.0 * robot.get_number_of_joints())


if __name__ == '__main__':
    unittest.main()
