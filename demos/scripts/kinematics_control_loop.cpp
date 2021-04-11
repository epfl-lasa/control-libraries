#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointVelocities.hpp"
#include "dynamical_systems/Linear.hpp"
#include "robot_model/Model.hpp"
#include <chrono>
#include <thread>
#include <utility>

using namespace state_representation;
using namespace dynamical_systems;
using namespace robot_model;
using namespace std::chrono_literals;

std::pair<JointPositions, CartesianPose> control_loop_step(Model& robot,
                                                           const JointPositions& current_joint_positions,
                                                           const Linear<CartesianState>& ds,
                                                           const std::chrono::nanoseconds& dt) {
  // get current position of the eef
  CartesianPose current_eef_pose = robot.forward_geometry(current_joint_positions);
  // get the twist evaluated at current pose
  CartesianTwist desired_twist = ds.evaluate(current_eef_pose);
  // apply the inverse kinematics
  JointVelocities desired_joint_velocities = robot.inverse_kinematic(current_joint_positions, desired_twist);
  // integrate the new position
  JointPositions new_joint_positions = dt * desired_joint_velocities + current_joint_positions;
  CartesianPose new_eef_pose = robot.forward_geometry(new_joint_positions);
  // print the new state and pose
  std::cout << new_joint_positions << std::endl;
  std::cout << new_eef_pose << std::endl;
  // return it
  return std::make_pair(new_joint_positions, new_eef_pose);
}

void control_loop(Model& robot, const std::chrono::nanoseconds& dt, double tolerance) {
  // set a desired target for the robot eef and a linear ds toward the target
  CartesianPose target("panda_link8");
  target.set_position(.5, .0, .5);
  Linear<CartesianState> linear_ds(target);
  // set a joint configuration
  JointPositions current_joint_positions = JointPositions::Zero("franka", robot.get_joint_frames());
  // loop until target is reached
  double distance;
  do {
    auto current_states = control_loop_step(robot, current_joint_positions, linear_ds, dt);
    current_joint_positions = std::get<JointPositions>(current_states);
    distance = dist(std::get<CartesianPose>(current_states), target, CartesianStateVariable::POSE);
    std::cout << "distance to attractor: " << std::to_string(distance) << std::endl;
    std::cout << "-----------" << std::endl;
    std::this_thread::sleep_for(dt);
  } while (distance > tolerance);

  auto current_states = control_loop_step(robot, current_joint_positions, linear_ds, dt);
  std::cout << "##### TARGET #####" << std::endl;
  std::cout << target << std::endl;
  std::cout << "##### CURRENT STATES #####" << std::endl;
  std::cout << std::get<JointPositions>(current_states) << std::endl;
  std::cout << std::get<CartesianPose>(current_states) << std::endl;
}

int main(int, char**) {
  std::string robot_name = "franka";
  std::string urdf_path = std::string(SCRIPT_FIXTURES) + "panda_arm.urdf";
  Model robot(robot_name, urdf_path);
  auto dt = 10ms;
  control_loop(robot, dt, 1e-3);
}