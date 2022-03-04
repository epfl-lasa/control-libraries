import os
import unittest

import numpy as np
from robot_model import Model
from state_representation import JointState, JointPositions, JointTorques


class TestRobotModelDynamics(unittest.TestCase):
    robot_model = None
    joint_state = None
    test_gravity_expects = None
    test_coriolis_expects = None
    test_inertia_expects = None
    tol = None

    @classmethod
    def setUpClass(cls):
        cls.robot_model = Model("franka", os.path.join(os.path.dirname(os.path.realpath(__file__)), "panda_arm.urdf"))
        joint_state = JointState(cls.robot_model.get_robot_name(), cls.robot_model.get_number_of_joints())
        joint_state.set_positions([-1.957518, 1.037530, -1.093933, -1.485144, -1.937432, 2.251972, -1.373487])
        joint_state.set_velocities([0.308158, 0.378429, 0.496303, -0.098917, -0.832357, -0.542046, 0.826675])
        joint_state.set_accelerations([-0.695244, 0.651634, 0.076685, 0.992269, -0.843649, -0.114643, -0.786694])
        cls.joint_state = joint_state
        cls.test_gravity_expects = [-0.000000, -36.985759, -18.785455, 11.384067, -0.532842, -1.346848, -0.071370]
        cls.test_coriolis_expects = [-0.049325, -0.064489, -0.060717, -0.295963, -0.009998, 0.026109, 0.003887]
        cls.test_inertia_expects = [-0.733398, 0.568876, -0.066627, -0.156614, -0.030068, -0.025849, -0.006774]
        cls.tol = 1e-5

    def test_compute_inertia_matrix(self):
        joint_positions = JointPositions().Random("robot", 7)
        inertia = self.robot_model.compute_inertia_matrix(joint_positions)
        self.assertTrue(inertia.shape[0] == 7 and inertia.shape[1] == 7)
        self.assertTrue(np.all(np.abs(inertia - inertia.T) < self.tol))

    def test_compute_inertia_torques(self):
        joint_state = JointState().Random("robot", 7)
        inertia_torques = self.robot_model.compute_inertia_torques(joint_state)
        self.assertTrue(isinstance(inertia_torques, JointTorques))
        self.assertTrue(np.linalg.norm(inertia_torques.data()) > 0)

        inertia_torques = self.robot_model.compute_inertia_torques(self.joint_state)
        [self.assertAlmostEqual(inertia_torques.get_torques()[i], self.test_inertia_expects[i], delta=self.tol) for i in
         range(7)]

    def test_compute_coriolis_matrix(self):
        joint_state = JointState().Random("robot", 7)
        coriolis = self.robot_model.compute_coriolis_matrix(joint_state)
        self.assertTrue(coriolis.shape[0] == 7 and coriolis.shape[1] == 7)

    def test_compute_coriolis_torques(self):
        joint_state = JointState().Random("robot", 7)
        coriolis_torques = self.robot_model.compute_coriolis_torques(joint_state)
        self.assertTrue(isinstance(coriolis_torques, JointTorques))
        self.assertTrue(np.linalg.norm(coriolis_torques.data()) > 0)

        coriolis_torques = self.robot_model.compute_coriolis_torques(self.joint_state)
        [self.assertAlmostEqual(coriolis_torques.get_torques()[i], self.test_coriolis_expects[i], delta=self.tol) for i
         in range(7)]

    def test_gravity_torques(self):
        joint_positions = JointPositions().Random("robot", 7)
        gravity_torques = self.robot_model.compute_gravity_torques(joint_positions)
        self.assertTrue(isinstance(gravity_torques, JointTorques))
        self.assertTrue(np.linalg.norm(gravity_torques.data()) > 0)

        joint_positions = JointPositions(self.joint_state)
        gravity_torques = self.robot_model.compute_gravity_torques(joint_positions)
        [self.assertAlmostEqual(gravity_torques.get_torques()[i], self.test_gravity_expects[i], delta=self.tol) for i
         in range(7)]


if __name__ == '__main__':
    unittest.main()
