#include <chrono>
#include <thread>
#include <utility>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/robot/JointPositions.hpp>
#include <state_representation/robot/JointVelocities.hpp>
#include <dynamical_systems/Linear.hpp>
#include <robot_model/Model.hpp>

using namespace state_representation;
using namespace dynamical_systems;
using namespace robot_model;
using namespace std::chrono_literals;

namespace {
class DummyRobotInterface {
private:
  Model robot_; ///< robot model to compute the kinematics

public:
  JointPositions joint_positions; ///< current joint positions of the robot
  CartesianPose eef_pose; ///< current eef pose of the robot

  explicit DummyRobotInterface(const std::string& robot_name, const std::string& urdf_path) :
      robot_(robot_name, urdf_path) {
    this->joint_positions = JointPositions::Zero(robot_name, this->robot_.get_joint_frames());
    this->read_robot_state();
  }

  void read_robot_state() {
    // this is a dummy robot, we assume the joint state is executed but add a bit of noise
    this->joint_positions += 0.001 * JointPositions::Random(this->joint_positions.get_name(), this->robot_.get_joint_frames());
    this->eef_pose = this->robot_.forward_kinematics(this->joint_positions);
  }

  void send_control_command(const CartesianTwist& desired_eef_twist, const std::chrono::nanoseconds& dt) {
    // apply the inverse kinematics
    JointVelocities desired_joint_velocities = this->robot_.inverse_velocity(desired_eef_twist, this->joint_positions);
    // integrate the new position
    this->joint_positions = dt * desired_joint_velocities + this->joint_positions;
  }
};

void control_loop_step(DummyRobotInterface& robot,
                       const Linear<CartesianState>& ds,
                       const std::chrono::nanoseconds& dt) {
  // read the robot state
  robot.read_robot_state();
  // print the state and eef pose
  std::cout << robot.joint_positions << std::endl;
  std::cout << robot.eef_pose << std::endl;
  // get the twist evaluated at current pose
  CartesianTwist desired_twist = ds.evaluate(robot.eef_pose);
  // send the desired twist to the robot
  robot.send_control_command(desired_twist, dt);
}

void control_loop(DummyRobotInterface& robot, const std::chrono::nanoseconds& dt, double tolerance) {
  // set a desired target for the robot eef and a linear ds toward the target
  CartesianPose target(robot.eef_pose.get_name(), robot.eef_pose.get_reference_frame());
  target.set_position(.5, .0, .5);
  Linear<CartesianState> linear_ds(target);

  double distance;
  do {
    control_loop_step(robot, linear_ds, dt);
    distance = dist(robot.eef_pose, target, CartesianStateVariable::POSE);
    std::cout << "distance to attractor: " << std::to_string(distance) << std::endl;
    std::cout << "-----------" << std::endl;
    std::this_thread::sleep_for(dt);
  } while (distance > tolerance);

  std::cout << "##### TARGET #####" << std::endl;
  std::cout << target << std::endl;
  std::cout << "##### CURRENT STATES #####" << std::endl;
  std::cout << robot.joint_positions << std::endl;
  std::cout << robot.eef_pose << std::endl;
}
}

int main(int, char**) {
  std::string robot_name = "franka";
  std::string urdf_path = std::string(SCRIPT_FIXTURES) + "panda_arm.urdf";
  DummyRobotInterface robot("franka", urdf_path);
  auto dt = 10ms;
  control_loop(robot, dt, 1e-3);
}