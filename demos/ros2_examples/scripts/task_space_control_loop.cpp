#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <dynamical_systems/Linear.hpp>

using namespace state_representation;
using namespace dynamical_systems;
using namespace std::chrono_literals;

namespace {
class TaskSpaceControlLoop : public rclcpp::Node {
private:
  std::chrono::nanoseconds dt_; ///< period for the control loop
  double distance_tolerance_; ///< tolerance to the attractor to stop the control loop
  std::thread run_thread_; ///< thread object to start the main loop, i.e. the run function, in parallel of the rest
  CartesianPose current_pose_; ///< current pose
  CartesianPose target_; ///< attractor pose
  std::shared_ptr<Linear < CartesianState>> dynamical_system_ = nullptr; ///< dynamical system to the attractor

  void send_transform(const CartesianPose& pose, const std::string& pose_name_prefix = "") {
    auto node_handle = this->shared_from_this();
    static tf2_ros::TransformBroadcaster br(node_handle);
    geometry_msgs::msg::TransformStamped pose_tf;
    pose_tf.header.stamp = node_handle->now();
    pose_tf.header.frame_id = pose.get_reference_frame();
    std::string prefix = (pose_name_prefix != "") ? pose_name_prefix + "_" : "";
    pose_tf.child_frame_id = prefix + pose.get_name();
    // populated the transform
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(pose.get_position()(0), pose.get_position()(1), pose.get_position()(2)));
    transform.setRotation(tf2::Quaternion(pose.get_orientation().x(),
                                          pose.get_orientation().y(),
                                          pose.get_orientation().z(),
                                          pose.get_orientation().w()));
    pose_tf.transform = tf2::toMsg(transform);
    br.sendTransform(pose_tf);
  }

public:
  explicit TaskSpaceControlLoop(const std::string& node_name,
                                const std::chrono::nanoseconds& dt,
                                double distance_tolerance) :
      Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false)),
      dt_(dt),
      distance_tolerance_(distance_tolerance) {
    this->init();
  }

  void init() {
    // set a desired target and a linear ds toward the target
    this->target_ = CartesianPose::Random("frame");
    this->dynamical_system_ = std::make_shared<Linear < CartesianState>>
    (this->target_);
    // set a starting pose
    this->current_pose_ = CartesianPose::Random("frame");
    // add the run thread
    std::function<void(void)> run_fnc = std::bind(&TaskSpaceControlLoop::run, this);
    this->run_thread_ = std::thread(run_fnc);
  }

  void step() {
    // get the twist evaluated at current pose
    CartesianTwist desired_twist = this->dynamical_system_->evaluate(this->current_pose_);
    // integrate the twist and add it to the current pose
    this->current_pose_ = this->dt_ * desired_twist + this->current_pose_;
  }

  void run() {
    // loop until target is reached
    double distance;
    do {
      auto start = std::chrono::steady_clock::now();
      this->step();
      distance = dist(this->current_pose_, this->target_, CartesianStateVariable::POSE);
      this->send_transform(this->target_, "attractor");
      this->send_transform(this->current_pose_);
      auto end = std::chrono::steady_clock::now();
      auto elapsed = end - start;
      auto timeToWait = this->dt_ - elapsed;
      if (timeToWait > std::chrono::nanoseconds::zero()) {
        std::this_thread::sleep_for(timeToWait);
      }
      RCLCPP_INFO(this->get_logger(), "Distance to attractor: %f", distance);
    } while (distance > this->distance_tolerance_ && rclcpp::ok());
  }
};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto dt = 10ms;
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<TaskSpaceControlLoop>("task_space_control_loop", dt, 1e-3);
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}