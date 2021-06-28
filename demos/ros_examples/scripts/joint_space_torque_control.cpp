#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/robot/JointTorques.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <dynamical_systems/Linear.hpp>
#include <controllers/impedance/CartesianTwistController.hpp>
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
  JointState joint_state_;

public:
  bool state_received = false;

  explicit RobotInterface(ros::NodeHandle* node_handle, const std::string& robot_name, const std::string& urdf_path) :
      robot_(robot_name, urdf_path) {
    subscriber_ = node_handle->subscribe("joint_states", 10, &RobotInterface::robot_state_callback, this);
    publisher_ = node_handle->advertise<std_msgs::Float64MultiArray>("effort_controller/command", 10, false);
    joint_state_ = JointState(robot_name, robot_.get_joint_frames());
  }

private:
  void robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!state_received) {
      state_received = true;
    }
    joint_state_.set_positions(msg->position);
    joint_state_.set_velocities(msg->velocity);
    joint_state_.set_torques(msg->effort);
  }

public:
  void publish(const JointTorques& command) {
    std_msgs::Float64MultiArray msg;
    msg.data = command.to_std_vector();
    publisher_.publish(msg);
  }

  CartesianPose get_eef_pose() {
    return robot_.forward_kinematics(joint_state_);
  }

  Jacobian get_jacobian() {
    return robot_.compute_jacobian(joint_state_);
  }

  const std::string& get_robot_name() {
    return robot_.get_robot_name();
  }

  const std::vector<std::string> get_robot_frames() {
    return robot_.get_frames();
  }
};
}

void control_loop(RobotInterface& robot, const int& freq) {
  // set a desired target and a linear ds toward the target
  CartesianPose target(robot.get_robot_frames().back(), robot.get_robot_frames().front());
  target.set_position(.5, .0, .5);
  target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Linear<CartesianState> linear_ds(target, gains);

  controllers::impedance::CartesianTwistController ctrl(2, 2, .2, .2);

  ros::Rate rate(freq);
  while (ros::ok()) {
    if (robot.state_received) {
      CartesianTwist twist = linear_ds.evaluate(robot.get_eef_pose());
      twist.clamp(0.25, 0.5);
      JointTorques command = ctrl.compute_command(twist, robot.get_eef_pose(), robot.get_jacobian());
      robot.publish(command);
      rate.sleep();
    }
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_space_velocity_control_loop");
  ros::NodeHandle node_handle;

  std::string robot_description;
  if (!node_handle.getParam("robot_description", robot_description)) {
    ROS_ERROR("Could load parameter 'robot_description' from parameter server.");
    return -1;
  }

  std::string robot_name = ros::this_node::getNamespace();
  std::string urdf_path = std::string(SCRIPT_FIXTURES) + robot_name + ".urdf";
  Model::create_urdf_from_string(robot_description, urdf_path);
  RobotInterface robot(&node_handle, robot_name, urdf_path);

  int freq = 500;
  control_loop(robot, freq);
}