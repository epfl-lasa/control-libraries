#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <dynamical_systems/Linear.hpp>
#include <controllers/impedance/CartesianTwistController.hpp>
#include <robot_model/Model.hpp>

using namespace state_representation;
using namespace std::chrono_literals;

namespace {
class TorqueControl : public rclcpp::Node {
private:
  std::chrono::nanoseconds dt_;

  std::shared_ptr<dynamical_systems::Linear<CartesianState>> ds_;
  std::shared_ptr<controllers::impedance::CartesianTwistController> ctrl_;
  std::shared_ptr<robot_model::Model> robot_;
  JointState joint_state_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  bool state_received = false;

private:
  void robot_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!this->state_received) {
      this->state_received = true;
    }
    this->joint_state_.set_positions(msg->position);
    this->joint_state_.set_velocities(msg->velocity);
    this->joint_state_.set_torques(msg->effort);
  }

public:
  explicit TorqueControl(const std::string& node_name, const std::chrono::nanoseconds& dt) :
      Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false)), dt_(dt) {
    this->init();
  }

  void init() {
    std::string robot_name = "franka";
    auto parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(this, "/" + robot_name + "/robot_state_publisher");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto urdf_string = parameters_client->get_parameter<std::string>("robot_description");

    std::string urdf_path = std::string(SCRIPT_FIXTURES) + robot_name + ".urdf";
    robot_model::Model::create_urdf_from_string(urdf_string, urdf_path);
    this->robot_ = std::make_shared<robot_model::Model>(robot_name, urdf_path);
    this->joint_state_ = JointState(this->robot_->get_robot_name(), this->robot_->get_joint_frames());

    CartesianPose target(this->robot_->get_frames().back(), this->robot_->get_frames().front());
    target.set_position(.6, .0, .5);
    target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
    std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    this->ds_ = std::make_shared<dynamical_systems::Linear<CartesianState>>(target, gains);

    this->ctrl_ = std::make_shared<controllers::impedance::CartesianTwistController>(10, 10, 1, 1);

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/" + robot_name + "/joint_states", 10,
        std::bind(&TorqueControl::robot_state_callback, this, std::placeholders::_1));
    this->publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/" + robot_name + "/torque_controller/command", 10);

    this->timer_ = this->create_wall_timer(dt_, std::bind(&TorqueControl::run, this));
  }

  void run() {
    if (this->state_received) {
      CartesianTwist twist = this->ds_->evaluate(this->get_eef_pose());
      twist.clamp(0.5, 0.5);
      JointTorques command = this->ctrl_->compute_command(twist, this->get_eef_pose(), this->get_jacobian());
      this->publish(command);
    }
  }

  void publish(const JointTorques& command) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = command.to_std_vector();
    publisher_->publish(msg);
  }

  CartesianPose get_eef_pose() {
    return this->robot_->forward_kinematics(this->joint_state_);
  }

  Jacobian get_jacobian() {
    return this->robot_->compute_jacobian(this->joint_state_);
  }
};
}

int main(int argc, char** argv) {
  auto dt = 4ms;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueControl>("joint_space_control_loop", dt));
  rclcpp::shutdown();
  return 0;
}