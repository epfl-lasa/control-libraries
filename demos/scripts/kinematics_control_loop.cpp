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

namespace {
class DummyRobotInterface {
private:
  Model robot_; ///< robot model to compute the kinematics

public:
  JointState joint_state_; ///< current joint state of the robot
  CartesianState eef_state_; ///< current eef state of the robot

  explicit DummyRobotInterface(const std::string& robot_name, const std::string& urdf_path) :
      robot_(robot_name, urdf_path) {
    this->joint_state_ = JointState::Zero(robot_name, this->robot_.get_joint_frames());
    this->read_robot_state();
  }

  void read_robot_state() {
    // this is a dummy robot, we assume the joint state is executed but add a bit of noise
    this->joint_state_ += 0.001 * JointState::Random(this->joint_state_.get_name(), this->robot_.get_joint_frames());
    this->eef_state_ = this->robot_.forward_geometry(this->joint_state_);
  }

  void send_control_command(const CartesianTwist& desired_eef_twist, const std::chrono::nanoseconds& dt) {
    // apply the inverse kinematics
    JointVelocities desired_joint_velocities = this->robot_.inverse_kinematic(this->joint_state_, desired_eef_twist);
    // integrate the new position
    this->joint_state_ = dt * desired_joint_velocities + this->joint_state_;
  }
};

void control_loop_step(DummyRobotInterface& robot,
                       const Linear<CartesianState>& ds,
                       const std::chrono::nanoseconds& dt) {
  // read the robot state
  robot.read_robot_state();
  // print the state and eef pose
  std::cout << robot.joint_state_ << std::endl;
  std::cout << robot.eef_state_ << std::endl;
  // get the twist evaluated at current pose
  CartesianTwist desired_twist = ds.evaluate(robot.eef_state_);
  // send the desired twist to the robot
  robot.send_control_command(desired_twist, dt);
}

void control_loop(DummyRobotInterface& robot, const std::chrono::nanoseconds& dt, double tolerance) {
  // set a desired target for the robot eef and a linear ds toward the target
  CartesianPose target(robot.eef_state_.get_name(), robot.eef_state_.get_reference_frame());
  target.set_position(.5, .0, .5);
  Linear<CartesianState> linear_ds(target);

  double distance;
  do {
    control_loop_step(robot, linear_ds, dt);
    distance = dist(robot.eef_state_, target, CartesianStateVariable::POSE);
    std::cout << "distance to attractor: " << std::to_string(distance) << std::endl;
    std::cout << "-----------" << std::endl;
    std::this_thread::sleep_for(dt);
  } while (distance > tolerance);

  std::cout << "##### TARGET #####" << std::endl;
  std::cout << target << std::endl;
  std::cout << "##### CURRENT STATES #####" << std::endl;
  std::cout << robot.joint_state_ << std::endl;
  std::cout << robot.eef_state_ << std::endl;
}
}

int main(int, char**) {
  std::string robot_name = "franka";
  std::string urdf_path = std::string(SCRIPT_FIXTURES) + "panda_arm.urdf";
  DummyRobotInterface robot("franka", urdf_path);
  auto dt = 10ms;
  control_loop(robot, dt, 1e-3);
}