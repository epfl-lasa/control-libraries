#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/robot/JointVelocities.hpp>
#include <dynamical_systems/Linear.hpp>
#include <robot_model/Model.hpp>

using namespace state_representation;
using namespace dynamical_systems;
using namespace robot_model;
using namespace std::chrono_literals;

namespace {
class RobotInterface {
private:
  Model robot_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

public:
  JointState joint_state;
  bool state_received = false;

  explicit RobotInterface(ros::NodeHandle* node_handle, const std::string& robot_name, const std::string& urdf_path) :
      robot_(robot_name, urdf_path) {
    this->subscriber_ = node_handle->subscribe("/joint_states", 1, &RobotInterface::robot_state_callback, this);
    this->publisher_ = node_handle->advertise<std_msgs::Float64MultiArray>("/velocity_controller/command", 10, false);
    joint_state = JointState(robot_name, robot_.get_joint_frames());
  }

private:
  void robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!state_received) {
      state_received = true;
    }
    joint_state.set_positions(msg->position);
    joint_state.set_velocities(msg->velocity);
    joint_state.set_torques(msg->effort);
  }

public:
  void publish(const JointVelocities& command) {
    std_msgs::Float64MultiArray msg;
    msg.data = command.to_std_vector();
    this->publisher_.publish(msg);
  }

  CartesianPose get_eef_pose() {
    return robot_.forward_kinematics(joint_state);
  }

  CartesianTwist get_eef_twist() {
    return robot_.forward_velocity(joint_state);
  }

  Jacobian get_jacobian() {
    return robot_.compute_jacobian(joint_state);
  }

  const std::string& get_robot_name() {
    return robot_.get_robot_name();
  }
};
}

void control_loop(RobotInterface& robot, const int& freq) {
  // set a desired target and a linear ds toward the target
  CartesianPose target("panda_link8", "panda_link0");
  target.set_position(.5, .0, .5);
  target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Linear<CartesianState> linear_ds(target, gains);

  ros::Rate rate(freq);
  while (ros::ok()) {
    if (robot.state_received) {
      CartesianTwist twist = linear_ds.evaluate(robot.get_eef_pose());
      twist.clamp(0.25, 0.5);
      JointVelocities command = robot.get_jacobian().solve(twist);
      robot.publish(command);
      rate.sleep();
    }
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_space_control_loop");
  ros::NodeHandle node_handle;

  std::string robot_name = "franka";
  std::string urdf_path = "/home/ros/ros_ws/install/" + std::string(SCRIPT_FIXTURES) + "panda_arm.urdf";
  RobotInterface robot(&node_handle, robot_name, urdf_path);

  int freq = 500;
  control_loop(robot, freq);
}