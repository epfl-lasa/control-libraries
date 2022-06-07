import math
import os
import state_representation as sr
import time
from datetime import timedelta
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from pyquaternion import Quaternion
from robot_model import Model, QPInverseVelocityParameters


class DummyRobotInterface:

    def __init__(self, robot_name, urdf_path):
        self._robot = Model(robot_name, urdf_path)
        self.eef_pose = None
        self.joint_positions = sr.JointPositions().Zero(robot_name, self._robot.get_joint_frames())
        self.read_robot_state()

    def read_robot_state(self):
        # this is a dummy robot, we assume the joint state is executed
        self.eef_pose = self._robot.forward_kinematics(self.joint_positions)

    def send_control_command(self, desired_eef_twist, dt):
        # create the inverse velocity parameters and set the period of the control loop
        parameters = QPInverseVelocityParameters()
        parameters.dt = dt
        # apply the inverse velocity
        desired_joint_velocities = self._robot.inverse_velocity(desired_eef_twist, self.joint_positions, parameters)
        # integrate the new position
        self.joint_positions = dt * desired_joint_velocities + self.joint_positions


def control_loop_step(robot, ds, dt):
    # read the robot state
    robot.read_robot_state()
    # print the state and eef pose
    print(robot.joint_positions)
    print(robot.eef_pose)
    # get the twist evaluated at current pose
    desired_twist = sr.CartesianTwist(ds.evaluate(robot.eef_pose))
    # send the desired twist to the robot
    robot.send_control_command(desired_twist, dt)


def control_loop(robot, dt, tolerance):
    target = sr.CartesianPose(robot.eef_pose.get_name(), robot.eef_pose.get_reference_frame())
    target.set_position(.5, .0, .75)
    target.set_orientation(Quaternion(axis=[.0, 1., .0], radians=math.pi))
    ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
    ds.set_parameter(sr.Parameter("attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE))

    # loop until target is reached
    distance = sr.dist(robot.eef_pose, target, sr.CartesianStateVariable.POSE)
    while distance > tolerance:
        control_loop_step(robot, ds, dt)
        distance = sr.dist(robot.eef_pose, target, sr.CartesianStateVariable.POSE)
        print(f"Distance to attractor: {distance}")
        print("-----------")
        time.sleep(dt.total_seconds())

    print("##### TARGET #####")
    print(target)
    print("##### CURRENT STATES #####")
    print(robot.joint_positions)
    print(robot.eef_pose)


def main():
    urdf_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "fixtures", "panda_arm.urdf")
    robot = DummyRobotInterface("franka", urdf_path)
    control_loop(robot, timedelta(milliseconds=100), 1e-3)


if __name__ == "__main__":
    main()
