import numpy as np
import os
import unittest
from datetime import timedelta
from robot_model import Model, InverseKinematicsParameters, QPInverseVelocityParameters
from state_representation import CartesianPose, CartesianTwist, JointState, JointPositions, JointVelocities


class TestRobotModelKinematics(unittest.TestCase):
    robot_model = None
    joint_state = None
    test_config = None
    test_jacobian_ee_expects = None
    test_fk_ee_expects = None
    test_fk_link4_expects = None
    test_velocity_fk_expects = None
    tol = None

    @classmethod
    def setUpClass(cls):
        cls.robot_model = Model("franka", os.path.join(os.path.dirname(os.path.realpath(__file__)), "panda_arm.urdf"))
        joint_state = JointState(cls.robot_model.get_robot_name(), cls.robot_model.get_joint_frames())
        cls.joint_state = joint_state
        test_config = JointState(cls.robot_model.get_robot_name(), cls.robot_model.get_joint_frames())
        test_config.set_positions([-1.957518, 1.037530, -1.093933, -1.485144, -1.937432, 2.251972, -1.373487])
        test_config.set_velocities([0.308158, 0.378429, 0.496303, -0.098917, -0.832357, -0.542046, 0.826675])
        cls.test_config = test_config
        cls.test_jacobian_ee_expects = np.array([0.275154, -0.005914, 0.127368, -0.041238, 0.003824, 0.018324, 0.000000,
                                                 -0.693174, -0.014523, -0.347282, -0.422943, 0.026691, -0.034385,
                                                 0.000000,
                                                 -0.000000, -0.516268, -0.463478, 0.313197, -0.006376, -0.132947,
                                                 0.000000,
                                                 -0.000000, 0.926150, -0.324787, -0.254761, -0.935275, -0.138023,
                                                 -0.842118,
                                                 0.000000, -0.377155, -0.797555, 0.591396, 0.050315, -0.963323,
                                                 0.236446,
                                                 1.000000, 0.000000, 0.508349, 0.765080, -0.350326, 0.230127,
                                                 0.484697]).reshape(6, 7)
        cls.test_fk_ee_expects = CartesianPose("ee", [-0.693174, -0.275154, 0.348681],
                                               [0.548764, -0.464146, -0.205476, 0.664234],
                                               cls.robot_model.get_base_frame())
        cls.test_fk_link4_expects = CartesianPose("link4", [-0.177776, -0.242212, 0.461029],
                                                  [0.717822, -0.327979, 0.099446, 0.606030],
                                                  cls.robot_model.get_base_frame())
        cls.test_velocity_fk_expects = CartesianTwist("ee",
                                                      [0.136730, -0.353202, -0.379006, 0.371630, 0.078694, 1.052318],
                                                      cls.robot_model.get_base_frame())
        cls.tol = 1e-3

    def assert_np_array_equal(self, a: np.array, b: np.array, places=3):
        try:
            np.testing.assert_almost_equal(a, b, decimal=places)
        except AssertionError as e:
            self.fail(f'{e}')

    def test_fk_joint_state_size(self):
        dummy = JointPositions(self.robot_model.get_robot_name(), 6)
        self.assertRaises(ValueError, self.robot_model.forward_kinematics, dummy)

    def test_fk_invalid_frame(self):
        self.assertRaises(ValueError, self.robot_model.forward_kinematics, JointPositions(self.joint_state),
                          "panda_link99")

    def test_fk_ee(self):
        self.assert_np_array_equal(self.robot_model.forward_kinematics(JointPositions(self.joint_state)).get_position(),
                                   self.robot_model.forward_kinematics(JointPositions(self.joint_state),
                                                                       "panda_link8").get_position())

    def test_fk(self):
        ee_pose = self.robot_model.forward_kinematics(JointPositions(self.test_config))
        self.assertTrue(ee_pose.dist(self.test_fk_ee_expects) < 1e-3)
        link4_pose = self.robot_model.forward_kinematics(JointPositions(self.test_config), "panda_link4")
        self.assertTrue(link4_pose.dist(self.test_fk_link4_expects) < self.tol)

    def test_velocity_fk(self):
        ee_twist = self.robot_model.forward_velocity(self.test_config)
        self.assertTrue(ee_twist.dist(self.test_velocity_fk_expects) < self.tol)

    def test_ik(self):
        config = JointPositions("robot", self.robot_model.get_joint_frames(),
                                [-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983])
        param = InverseKinematicsParameters()
        param.tolerance = self.tol
        dt = timedelta(seconds=1)

        reference = self.robot_model.forward_kinematics(config, "panda_link8")
        q = self.robot_model.inverse_kinematics(reference, param, "panda_link8")
        X = self.robot_model.forward_kinematics(q, "panda_link8")
        self.assertTrue(max(abs(((reference - X) / dt).data())) < self.tol)

    def test_ik_no_convergence(self):
        config = JointPositions("robot", self.robot_model.get_joint_frames(),
                                [-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983])
        param = InverseKinematicsParameters()
        param.max_number_of_iterations = 1

        reference = self.robot_model.forward_kinematics(config, "panda_link8")
        self.assertRaises(RuntimeError, self.robot_model.inverse_kinematics, reference, param, "panda_link8")

    def test_inverse_velocity(self):
        eef_frame = self.robot_model.get_frames()[-1]
        des_ee_twist = CartesianTwist.Random(eef_frame, self.robot_model.get_base_frame())
        joint_velocities = self.robot_model.inverse_velocity(des_ee_twist, JointPositions(self.test_config))

        state = JointState(self.test_config)
        state.set_velocities(joint_velocities.data())
        act_ee_twist = self.robot_model.forward_velocity(state)
        self.assertTrue(des_ee_twist.dist(act_ee_twist) < self.tol)

        joint_velocities2 = self.robot_model.inverse_velocity(des_ee_twist, JointPositions(self.test_config),
                                                              QPInverseVelocityParameters())
        state2 = JointState(self.test_config)
        state.set_velocities(joint_velocities2.data())
        act_ee_twist2 = self.robot_model.forward_velocity(state2)
        # FIXME: this doesn't pass (same in cpp)
        # self.assertTrue(des_ee_twist.dist(act_ee_twist2) < self.tol)

    def test_inverse_velocity_constraints(self):
        eef_frame = self.robot_model.get_frames()[-1]
        parameters = QPInverseVelocityParameters()
        parameters.linear_velocity_limit = 0.1
        parameters.angular_velocity_limit = 0.2

        des_ee_twist = CartesianTwist(eef_frame, [1, 0, 0], [1, 0, 0], self.robot_model.get_base_frame())
        joint_velocities = self.robot_model.inverse_velocity(des_ee_twist, JointPositions(self.test_config), parameters)
        state = JointState(self.test_config)
        state.set_velocities(joint_velocities.data())
        act_ee_twist = self.robot_model.forward_velocity(state)

        self.assertTrue(np.linalg.norm(act_ee_twist.get_linear_velocity()) <= 0.1)
        self.assertTrue(np.linalg.norm(act_ee_twist.get_angular_velocity()) <= 0.2)

    def test_compute_jacobian(self):
        jac = self.robot_model.compute_jacobian(JointPositions(self.test_config))
        self.assert_np_array_equal(jac.data(), self.test_jacobian_ee_expects)

    def test_compute_jacobian_time_derivative(self):
        velocities = JointVelocities(self.test_config)
        pos1 = JointPositions(self.test_config)
        pos2 = pos1 + timedelta(milliseconds=1) * velocities
        jac1 = self.robot_model.compute_jacobian(pos1)
        jac2 = self.robot_model.compute_jacobian(pos2)

        jac1_dt = self.robot_model.compute_jacobian_time_derivative(pos1, velocities)
        jac2_expect = jac1.data() + 0.001 * jac1_dt
        self.assert_np_array_equal(jac2.data(), jac2_expect)

        velocities.set_zero()
        jt = self.robot_model.compute_jacobian_time_derivative(pos1, velocities)
        self.assertTrue(np.sum(jt) < self.tol)


if __name__ == '__main__':
    unittest.main()
