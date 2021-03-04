#include "robot_model/Model.hpp"

#include <stdexcept>
#include <gtest/gtest.h>

#include "robot_model/Exceptions/InvalidJointStateSizeException.hpp"
#include "robot_model/Exceptions/FrameNotFoundException.hpp"

using namespace RobotModel;

class RobotModelTest : public testing::Test {
protected:
  void SetUp() override {
    robot_name = "franka";
    urdf = std::string(TEST_FIXTURES) + "/panda_arm.urdf";
    franka = Model(robot_name, urdf);
    joint_state = StateRepresentation::JointState(robot_name, 7);
  }

  Model empty = Model();
  Model franka;
  std::string robot_name;
  std::string urdf;

  StateRepresentation::JointState joint_state;
};

TEST_F(RobotModelTest, TestInitEmptyModel) {
  EXPECT_THROW(empty.init_model(), std::invalid_argument);
}

TEST_F(RobotModelTest, TestSetName) {
  empty.set_robot_name(robot_name);
  EXPECT_EQ(empty.get_robot_name(), robot_name);
}

TEST_F(RobotModelTest, TestSetUrdfPath) {
  empty.set_urdf_path(urdf);
  EXPECT_EQ(empty.get_urdf_path(), urdf);
}

TEST_F(RobotModelTest, TestInitModel) {
  empty.set_robot_name(robot_name);
  empty.set_urdf_path(urdf);
  EXPECT_NO_THROW(empty.init_model());
}

TEST_F(RobotModelTest, TestConstructor) {
  Model tmp(robot_name, urdf);
  EXPECT_NO_THROW(franka = tmp);
}

TEST_F(RobotModelTest, TestNumberOfJoints) {
  EXPECT_EQ(franka.get_number_of_joints(), 7);
}

TEST_F(RobotModelTest, TestForwardGeometryJointStateSize) {
  StateRepresentation::JointState dummy = StateRepresentation::JointState(robot_name, 6);
  EXPECT_THROW(franka.forward_geometry(dummy), Exceptions::InvalidJointStateSizeException);
}

TEST_F(RobotModelTest, TestForwardGeometry) {
  EXPECT_EQ(franka.forward_geometry(joint_state).get_position(),
            franka.forward_geometry(joint_state, "panda_link8").get_position());
}

TEST_F(RobotModelTest, TestForwardGeometryInvalidFrameName) {
  EXPECT_THROW(franka.forward_geometry(joint_state, "panda_link99"), Exceptions::FrameNotFoundException);
}

TEST_F(RobotModelTest, TestJacobianJointNames) {
  StateRepresentation::JointState dummy = StateRepresentation::JointState(robot_name, 7);
  StateRepresentation::Jacobian jac = franka.compute_jacobian(dummy);
  for (int i = 0; i < 7; ++i) {
    std::string jname = "panda_joint" + std::to_string(i + 1);
    EXPECT_TRUE(jname.compare(jac.get_joint_names()[i]) == 0);
  }
}

TEST_F(RobotModelTest, TestJacobianInvalidFrameName) {
  EXPECT_THROW(franka.compute_jacobian(joint_state, "panda_link99"), Exceptions::FrameNotFoundException);
}

TEST_F(RobotModelTest, TestJacobianNbRows) {
  StateRepresentation::Jacobian jac = franka.compute_jacobian(joint_state, "panda_joint2");
  EXPECT_EQ(jac.get_nb_rows(), 6);
}

TEST_F(RobotModelTest, TestJacobianNbCols) {
  StateRepresentation::Jacobian jac = franka.compute_jacobian(joint_state, "panda_joint2");
  EXPECT_EQ(jac.get_nb_cols(), joint_state.get_size());
}

TEST_F(RobotModelTest, TestComputeCoriolisMatrix) {
  StateRepresentation::JointState js = StateRepresentation::JointState::Random("robot", 7);
  Eigen::MatrixXd coriolis = franka.compute_coriolis_matrix(js);
  EXPECT_TRUE(coriolis.rows() == js.get_size() && coriolis.cols() == js.get_size());
}

TEST_F(RobotModelTest, TestComputeCoriolisForces) {
  StateRepresentation::JointState js = StateRepresentation::JointState::Random("robot", 7);
  StateRepresentation::JointTorques coriolis_forces = franka.compute_coriolis_forces(js);
  EXPECT_TRUE(coriolis_forces.data().norm() > 0);
}

TEST_F(RobotModelTest, TestComputeGravityTorques) {
  StateRepresentation::JointPositions jp = StateRepresentation::JointPositions::Random("robot", 7);
  StateRepresentation::JointTorques gravity_torques = franka.compute_gravity_torques(jp);
  EXPECT_TRUE(gravity_torques.data().norm() > 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}