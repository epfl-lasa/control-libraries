import os
import unittest

import numpy as np
from robot_model import Model, create_urdf_from_string
from state_representation import JointPositions, Jacobian

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
    'compute_jacobian',
    'compute_jacobian_time_derivative',
    'compute_inertia_matrix',
    'compute_inertia_torques',
    'compute_coriolis_matrix',
    'compute_coriolis_torques',
    'compute_gravity_torques',
    'print_qp_problem',
]


class TestRobotModel(unittest.TestCase):
    robot_model = None
    robot_name = None
    urdf_path = None
    create_urdf_test_path = None
    joint_positions = None
    tol = None

    @classmethod
    def setUpClass(cls):
        cls.robot_name = "franka"
        cls.urdf_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "panda_arm.urdf")
        cls.robot_model = Model(cls.robot_name, cls.urdf_path)
        cls.joint_positions = JointPositions(cls.robot_name, cls.robot_model.get_number_of_joints())
        cls.create_urdf_test_path = "/tmp/urdf_test.urdf"
        cls.tol = 1e-5

    @classmethod
    def tearDownClass(cls):
        try:
            os.remove(cls.create_urdf_test_path)
        except OSError as ex:
            print(ex)

    def test_callable_methods(self):
        methods = [m for m in dir(Model) if callable(getattr(Model, m))]
        for expected in ROBOT_MODEL_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_get_set_name(self):
        self.assertEqual(self.robot_model.get_robot_name(), self.robot_name)
        self.robot_model.set_robot_name("robot")
        self.assertEqual(self.robot_model.get_robot_name(), "robot")

    def test_get_urdf_path(self):
        self.assertEqual(self.robot_model.get_urdf_path(), self.urdf_path)

    def test_number_of_joints(self):
        self.assertEqual(self.robot_model.get_number_of_joints(), 7)

    def test_copy_constructor(self):
        tmp = Model(self.robot_model)
        self.assertEqual(tmp.get_robot_name(), self.robot_name)
        self.assertEqual(tmp.get_urdf_path(), self.urdf_path)
        self.assertEqual(tmp.get_number_of_joints(), 7)

    def test_jacobian_names(self):
        zero = JointPositions(self.robot_name, 7)
        jac = self.robot_model.compute_jacobian(zero)
        self.assertTrue(isinstance(jac, Jacobian))
        [self.assertEqual("panda_joint" + str(i + 1), jac.get_joint_names()[i]) for i in range(7)]

        self.assertEqual(jac.get_reference_frame(), "panda_link0")
        self.assertEqual(jac.get_frame(), "panda_link8")

        jac2 = self.robot_model.compute_jacobian(zero, "panda_link2")
        self.assertEqual(jac2.get_reference_frame(), "panda_link0")
        self.assertEqual(jac2.get_frame(), "panda_link2")

    def test_jacobian_invalid(self):
        dummy = JointPositions(self.robot_name, 6)
        with self.assertRaises(ValueError):
            self.robot_model.compute_jacobian(self.joint_positions, "panda_link99")
            self.robot_model.compute_jacobian(dummy, "panda_link8")

    def test_jacobian_size(self):
        jac = self.robot_model.compute_jacobian(self.joint_positions, "panda_link2")
        self.assertEqual(jac.cols(), self.joint_positions.get_size())
        self.assertEqual(jac.rows(), 6)

    def test_get_set_gravity(self):
        self.assertTrue(np.linalg.norm(np.array([0, 0, -9.81]) - self.robot_model.get_gravity_vector()) < self.tol)
        dummy_vector = np.random.rand(3)
        self.assertFalse(np.linalg.norm(dummy_vector - self.robot_model.get_gravity_vector()) < self.tol)
        self.robot_model.set_gravity_vector(dummy_vector)
        self.assertTrue(np.linalg.norm(dummy_vector - self.robot_model.get_gravity_vector()) < self.tol)

    def test_create_urdf_from_string(self):
        with open(self.urdf_path, "r") as file:
            content = file.read()
            self.assertTrue(create_urdf_from_string(content, self.create_urdf_test_path))
        model = Model("fromCreatedUrdf", self.create_urdf_test_path)
        self.assertEqual(model.get_number_of_joints(), self.robot_model.get_number_of_joints())
        self.assertEqual(len(model.get_frames()), len(self.robot_model.get_frames()))
        [self.assertEqual(model.get_frames()[i], self.robot_model.get_frames()[i]) for i in
         range(len(self.robot_model.get_frames()))]

        self.assertFalse(create_urdf_from_string("dummy_string", "/dev/null/invalid.urdf"))
        self.assertTrue(create_urdf_from_string("dummy_string", self.create_urdf_test_path))
        with self.assertRaises(Exception):
            Model("dummy", self.create_urdf_test_path)


if __name__ == '__main__':
    unittest.main()
