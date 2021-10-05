import unittest
import os

import numpy as np
from robot_model import Model

ROBOT_MODEL_METHOD_EXPECTS = [
    'get_robot_name',
    'set_robot_name',
    'get_urdf_path',
    'get_number_of_joints',
    'get_joint_frames',
    'get_frames',
    'get_base_frame',
    'get_gravity_vector',
    'set_gravity_vector',
    'print_qp_problem',
    'clamp_in_range',
]


class TestModel(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(Model) if callable(getattr(Model, m))]
        for expected in ROBOT_MODEL_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_constructors(self):
        robot = Model("test", os.path.join(os.path.dirname(os.path.realpath(__file__)), "panda_arm.urdf"))
        print(robot.get_robot_name())
        print(robot.get_urdf_path())
        print(robot.get_number_of_joints())
        print(robot.get_joint_frames())
        print(robot.get_frames())
        print(robot.get_base_frame())

if __name__ == '__main__':
    unittest.main()
