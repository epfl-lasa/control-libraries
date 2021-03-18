#include "robot_model/Model.hpp"

#include <stdexcept>
#include <memory>
#include <gtest/gtest.h>

#include "robot_model/exceptions/InvalidJointStateSizeException.hpp"
#include "robot_model/exceptions/FrameNotFoundException.hpp"

using namespace robot_model;

class RobotModelTest : public testing::Test {
protected:
  void SetUp() override {
    robot_name = "franka";
    urdf = std::string(TEST_FIXTURES) + "panda_arm.urdf";
    franka = std::make_unique<Model>(robot_name, urdf);
    joint_state = state_representation::JointState(robot_name, 7);
  }

  std::unique_ptr<Model> franka;
  std::string robot_name;
  std::string urdf;

  state_representation::JointState joint_state;

  double tol = 1e-5;
};

TEST_F(RobotModelTest, TestGetName) {
  EXPECT_EQ(franka->get_robot_name(), robot_name);
}

TEST_F(RobotModelTest, TestSetName) {
  franka->set_robot_name("robot");
  EXPECT_EQ(franka->get_robot_name(), "robot");
}

TEST_F(RobotModelTest, TestGetUrdfPath) {
  EXPECT_EQ(franka->get_urdf_path(), urdf);
}

TEST_F(RobotModelTest, TestCopyConstructor) {
  Model tmp(robot_name, urdf);
  EXPECT_NO_THROW(*franka = tmp);
}

TEST_F(RobotModelTest, TestNumberOfJoints) {
  EXPECT_EQ(franka->get_number_of_joints(), 7);
}

TEST_F(RobotModelTest, TestForwardGeometryJointStateSize) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 6);
  EXPECT_THROW(franka->forward_geometry(dummy), exceptions::InvalidJointStateSizeException);
}

TEST_F(RobotModelTest, TestForwardGeometry) {
  EXPECT_EQ(franka->forward_geometry(joint_state).get_position(),
            franka->forward_geometry(joint_state, "panda_link8").get_position());
}

TEST_F(RobotModelTest, TestForwardGeometryInvalidFrameName) {
  EXPECT_THROW(franka->forward_geometry(joint_state, "panda_link99"), exceptions::FrameNotFoundException);
}

TEST_F(RobotModelTest, TestJacobianJointNames) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 7);
  state_representation::Jacobian jac = franka->compute_jacobian(dummy);
  for (int i = 0; i < 7; ++i) {
    std::string jname = "panda_joint" + std::to_string(i + 1);
    EXPECT_TRUE(jname.compare(jac.get_joint_names()[i]) == 0);
  }
}

TEST_F(RobotModelTest, TestJacobianFrameNames) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 7);
  state_representation::Jacobian jac = franka->compute_jacobian(dummy);
  EXPECT_TRUE(jac.get_reference_frame() == "panda_link0");
  EXPECT_TRUE(jac.get_frame() == "panda_link8");
  state_representation::Jacobian jac2 = franka->compute_jacobian(dummy, "panda_link2");
  EXPECT_TRUE(jac2.get_reference_frame() == "panda_link0");
  EXPECT_TRUE(jac2.get_frame() == "panda_link2");
}

TEST_F(RobotModelTest, TestJacobianInvalidFrameName) {
  EXPECT_THROW(franka->compute_jacobian(joint_state, "panda_link99"), exceptions::FrameNotFoundException);
}

TEST_F(RobotModelTest, TestJacobianNbRows) {
  state_representation::Jacobian jac = franka->compute_jacobian(joint_state, "panda_joint2");
  EXPECT_EQ(jac.rows(), 6);
}

TEST_F(RobotModelTest, TestJacobianNbCols) {
  state_representation::Jacobian jac = franka->compute_jacobian(joint_state, "panda_joint2");
  EXPECT_EQ(jac.cols(), joint_state.get_size());
}

TEST_F(RobotModelTest, TestGravityGetterAndSetters) {
  Eigen::Vector3d dummy_vector = Eigen::Vector3d::Random();
  EXPECT_FALSE((dummy_vector - franka->get_gravity_vector()).norm() < 1e-4);
  // set new gravity as dummy_vector and expect equality
  franka->set_gravity_vector(dummy_vector);
  EXPECT_TRUE((dummy_vector - franka->get_gravity_vector()).norm() < 1e-4);
}