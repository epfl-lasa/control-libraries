#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <dynamical_systems/Linear.hpp>

using namespace state_representation;
using namespace dynamical_systems;
using namespace std::chrono_literals;

void send_transform(const CartesianPose& pose, const std::string& pose_name_prefix = "") {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.get_position()(0), pose.get_position()(1), pose.get_position()(2)));
  transform.setRotation(tf::Quaternion(pose.get_orientation().x(),
                                        pose.get_orientation().y(),
                                        pose.get_orientation().z(),
                                        pose.get_orientation().w()));
  std::string prefix = (pose_name_prefix != "") ? pose_name_prefix + "_" : "";
  br.sendTransform(tf::StampedTransform(transform,
                                        ros::Time::now(),
                                        pose.get_reference_frame(),
                                        prefix + pose.get_name()));
}

CartesianPose control_loop_step(const CartesianPose& pose,
                                const Linear<CartesianState>& ds,
                                const std::chrono::nanoseconds& dt) {
  // get the twist evaluated at current pose
  CartesianTwist desired_twist = ds.evaluate(pose);
  // integrate the twist and add it to the current pose
  CartesianPose new_pose = dt * desired_twist + pose;
  // return it
  return new_pose;
}

void control_loop(const std::chrono::nanoseconds& dt, double tolerance) {
  // set a desired target and a linear ds toward the target
  CartesianPose target = CartesianPose::Random("frame");
  Linear<CartesianState> linear_ds(target);
  // set a starting pose
  CartesianPose current_pose = CartesianPose::Random("frame");
  // loop until target is reached
  double distance;
  do {
    current_pose = control_loop_step(current_pose, linear_ds, dt);
    distance = dist(current_pose, target, CartesianStateVariable::POSE);
    send_transform(target, "attractor");
    send_transform(current_pose);
    std::this_thread::sleep_for(dt);
  } while (distance > tolerance && ros::ok());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_space_control_loop");
  auto dt = 10ms;
  control_loop(dt, 1e-3);
}