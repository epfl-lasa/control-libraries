#include "robot_model/Model.hpp"

#include <stdexcept>
#include <memory>
#include <gtest/gtest.h>

#include "robot_model/exceptions/InvalidJointStateSizeException.hpp"
#include "robot_model/exceptions/FrameNotFoundException.hpp"

using namespace robot_model;

class RobotModelKinematicsTest : public testing::Test {
protected:
  void SetUp() override {
    robot_name = "franka";
    urdf_path = std::string(TEST_FIXTURES) + "panda_arm.urdf";
    franka = std::make_unique<Model>(robot_name, urdf_path);
    joint_state = state_representation::JointState(robot_name, 7);
  }

  std::unique_ptr<Model> franka;
  std::string robot_name;
  std::string urdf_path;

  state_representation::JointState joint_state;

  double tol = 1e-5;

  std::vector<state_representation::JointState> test_configs;
  std::vector<std::vector<double>> test_fk_ee_expects;
  std::vector<std::vector<double>> test_fk_link4_expects;
  std::vector<std::vector<double>> test_velocity_fk_expects;

  void setTestConfigurations() {
    // Random test configuration 1:
    state_representation::JointState config1("robot", 7);
    config1.set_positions({-0.600782, -0.754768, 0.863711, -2.290588, 2.816429, 2.519369, 0.969137});
    config1.set_velocities({0.578712, -0.861191, -0.513509, 0.019743, 0.731455, 0.994781, 0.219667});
    test_configs.push_back(config1);

    // Expected results for configuration 1:
    test_fk_ee_expects.push_back({2.183865, 3.748338, -3.253473, 0.737074, -0.596918, -0.216162, -0.231700});
    test_fk_link4_expects.push_back({-1.120024, 1.656432, -1.164723, 0.988721, -0.079872, 0.123119, -0.029890});
    test_velocity_fk_expects.push_back({0.594465, -0.181112, -0.595054, -0.292841, 0.038273, 0.256638});

    // Random test configuration 2:
    state_representation::JointState config2("robot", 7);
    config2.set_positions({-2.762982, 1.430103, 1.665231, -1.545128, 1.475401, 2.285995, -0.451094});
    config2.set_velocities({-0.506830, -0.587386, -0.352971, -0.286343, -0.184000, 0.028808, 0.817589});
    test_configs.push_back(config2);

    // Expected results for configuration 2:
    test_fk_ee_expects.push_back({1.595838, -2.129081, -1.749960, 0.759958, 0.177980, 0.207359, -0.589736});
    test_fk_link4_expects.push_back({-0.856825, -1.723148, -1.006424, 0.902739, -0.040011, 0.369850, -0.216037});
    test_velocity_fk_expects.push_back({0.399660, 0.502696, 0.350962, -0.147135, 0.096426, 0.000348});

    // Random test configuration 3:
    state_representation::JointState config3("robot", 7);
    config3.set_positions({-2.219674, -1.128062, -0.178967, -2.134750, 2.503023, 3.291643, 2.835908});
    config3.set_velocities({-0.611246, 0.638815, 0.228419, -0.529544, 0.584228, -0.133137, -0.740637});
    test_configs.push_back(config3);

    // Expected results for configuration 3:
    test_fk_ee_expects.push_back({-1.559395, -6.599230, -0.723621, 0.798761, -0.565952, 0.147204, 0.141458});
    test_fk_link4_expects.push_back({0.578824, -0.846291, -1.952971, 0.947269, -0.119300, -0.095053, 0.281804});
    test_velocity_fk_expects.push_back({1.129130, -0.374429, -0.371274, -0.149756, -0.358528, -0.279180});
  }
};

TEST_F(RobotModelKinematicsTest, TestForwardGeometryJointStateSize) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 6);
  EXPECT_THROW(franka->forward_geometry(dummy), exceptions::InvalidJointStateSizeException);
}

TEST_F(RobotModelKinematicsTest, TestForwardGeometryEE) {
  EXPECT_EQ(franka->forward_geometry(joint_state).get_position(),
            franka->forward_geometry(joint_state, "panda_link8").get_position());
}

TEST_F(RobotModelKinematicsTest, TestForwardGeometryInvalidFrameName) {
  EXPECT_THROW(franka->forward_geometry(joint_state, "panda_link99"), exceptions::FrameNotFoundException);
}

TEST_F(RobotModelKinematicsTest, TestForwardGeometry) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::CartesianPose ee_pose = franka->forward_geometry(test_configs[config]);
    state_representation::CartesianPose link4_pose = franka->forward_geometry(test_configs[config], "panda_link4");
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_NEAR(ee_pose.get_position()[i], test_fk_ee_expects[config][i], tol);
      EXPECT_NEAR(link4_pose.get_position()[i], test_fk_link4_expects[config][i], tol);
    }
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_NEAR(ee_pose.get_orientation().coeffs()[i], test_fk_ee_expects[config][3+i], tol);
      EXPECT_NEAR(link4_pose.get_orientation().coeffs()[i], test_fk_link4_expects[config][3+i], tol);
    }
  }
}

TEST_F(RobotModelKinematicsTest, TestForwardKinematics) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::CartesianTwist ee_twist = franka->forward_kinematic(test_configs[config]);
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_NEAR(ee_twist.get_linear_velocity()[i], test_velocity_fk_expects[config][i], tol);
      EXPECT_NEAR(ee_twist.get_angular_velocity()[i], test_velocity_fk_expects[config][3 + i], tol);
    }
  }
}

TEST_F(RobotModelKinematicsTest, TestInverseKinematics) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::CartesianTwist ee_twist = franka->forward_kinematic(test_configs[config]);
    state_representation::JointVelocities joint_twist = franka->inverse_kinematic(test_configs[config], ee_twist);
    for (std::size_t joint = 0; joint < franka->get_number_of_joints(); ++joint) {
      EXPECT_NEAR(joint_twist.get_velocities()[joint], test_configs[config].get_velocities()[joint], tol);
    }
  }
}
