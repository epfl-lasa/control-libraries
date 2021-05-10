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
    urdf_path = std::string(TEST_FIXTURES) + "panda_arm.urdf";
    franka = std::make_unique<Model>(robot_name, urdf_path);
    joint_state = state_representation::JointState(robot_name, 7);

    create_urdf_test_path = std::string(TEST_FIXTURES) + "urdf_test.urdf";
  }

  void TearDown() override {
    std::remove(create_urdf_test_path.c_str());
  }

  std::unique_ptr<Model> franka;
  std::string robot_name;
  std::string urdf_path;
  std::string create_urdf_test_path;

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
  EXPECT_EQ(franka->get_urdf_path(), urdf_path);
}

TEST_F(RobotModelTest, TestCopyConstructor) {
  Model tmp(robot_name, urdf_path);
  EXPECT_NO_THROW(*franka = tmp);
}

TEST_F(RobotModelTest, TestNumberOfJoints) {
  EXPECT_EQ(franka->get_number_of_joints(), 7);
}


TEST_F(RobotModelTest, TestJacobianJointNames) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 7);
  state_representation::Jacobian jac = franka->compute_jacobian(dummy);
  for (int i = 0; i < 7; ++i) {
    std::string jname = "panda_joint" + std::to_string(i + 1);
    EXPECT_EQ(jname, jac.get_joint_names()[i]);
  }
}

TEST_F(RobotModelTest, TestJacobianFrameNames) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 7);
  state_representation::Jacobian jac = franka->compute_jacobian(dummy);
  EXPECT_EQ(jac.get_reference_frame(), "panda_link0");
  EXPECT_EQ(jac.get_frame(), "panda_link8");
  state_representation::Jacobian jac2 = franka->compute_jacobian(dummy, "panda_link2");
  EXPECT_EQ(jac2.get_reference_frame(), "panda_link0");
  EXPECT_EQ(jac2.get_frame(), "panda_link2");
}

TEST_F(RobotModelTest, TestJacobianInvalidFrameName) {
  EXPECT_THROW(franka->compute_jacobian(joint_state, "panda_link99"), exceptions::FrameNotFoundException);
}

TEST_F(RobotModelTest, TestJacobianInvalidJointStateSize) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 6);
  EXPECT_THROW(franka->compute_jacobian(dummy, "panda_link8"), exceptions::InvalidJointStateSizeException);
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

TEST_F(RobotModelTest, TestCreateURDFFromStringSuccess) {
  std::ifstream file(urdf_path);
  std::stringstream strStream;
  strStream << file.rdbuf();
  std::string urdf_string = strStream.str();
  EXPECT_TRUE(Model::create_urdf_from_string(urdf_string, create_urdf_test_path));
  Model fromCreatedUrdf = Model("fromCreatedUrdf", create_urdf_test_path);
  EXPECT_EQ(fromCreatedUrdf.get_number_of_joints(), franka->get_number_of_joints());
  ASSERT_EQ(fromCreatedUrdf.get_frames().size(), franka->get_frames().size());
  for (std::size_t frame = 0; frame < franka->get_frames().size(); ++frame) {
    EXPECT_EQ(fromCreatedUrdf.get_frames().at(frame), franka->get_frames().at(frame));
  }
}

TEST_F(RobotModelTest, TestCreateURDFFromStringFail) {
  EXPECT_FALSE(Model::create_urdf_from_string("dummy string", "/dev/null/invalid.urdf"));
  EXPECT_TRUE(Model::create_urdf_from_string("dummy string", create_urdf_test_path));
  EXPECT_ANY_THROW(Model("dummy", create_urdf_test_path));
}

TEST_F(RobotModelTest, TestModelGetter) {
  const pinocchio::Model& robot_model = franka->get_pinocchio_model();
  EXPECT_TRUE(robot_model.existBodyName("panda_link0"));
}