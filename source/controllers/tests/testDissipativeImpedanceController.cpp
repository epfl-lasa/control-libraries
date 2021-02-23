#include "controllers/impedance/Dissipative.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include "state_representation/Robot/JointVelocities.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include <numeric>
#include <gtest/gtest.h>

using namespace controllers::impedance;
using namespace StateRepresentation;

class DissipativeImpedanceControllerTest : public testing::Test {
protected:
  void SetUp() override {}

  void set_controller_space(const ComputationalSpaceType& computational_space) {
    task_controller_ = Dissipative<CartesianState>(computational_space);
  }

  Dissipative<CartesianState> task_controller_;
  Dissipative<JointState> joint_controller_ = Dissipative<JointState>(4);
  double tolerance_ = 1e-4;
};

TEST_F(DissipativeImpedanceControllerTest, TestCopyConstructorCartesian) {
  Dissipative<CartesianState> new_control = Dissipative<CartesianState>(ComputationalSpaceType::LINEAR);
  new_control.set_damping_eigenvalue(50, 2);
  Dissipative<CartesianState> copy(new_control);
  double tolerance = 1e-4;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(new_control.get_damping().col(i).norm(), copy.get_damping().col(i).norm(), tolerance);
    EXPECT_NEAR(new_control.get_stiffness().col(i).norm(), copy.get_stiffness().col(i).norm(), tolerance);
    EXPECT_NEAR(new_control.get_inertia().col(i).norm(), copy.get_inertia().col(i).norm(), tolerance);
  }
  EXPECT_NEAR(new_control.get_damping_eigenvalues().norm(), copy.get_damping_eigenvalues().norm(), tolerance);
}

TEST_F(DissipativeImpedanceControllerTest, TestAssignmentOperatorCartesian) {
  Dissipative<CartesianState> new_control = Dissipative<CartesianState>(ComputationalSpaceType::LINEAR);
  new_control.set_damping_eigenvalue(50, 2);
  Dissipative<CartesianState> copy = new_control;
  double tolerance = 1e-4;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(new_control.get_damping().col(i).norm(), copy.get_damping().col(i).norm(), tolerance);
    EXPECT_NEAR(new_control.get_stiffness().col(i).norm(), copy.get_stiffness().col(i).norm(), tolerance);
    EXPECT_NEAR(new_control.get_inertia().col(i).norm(), copy.get_inertia().col(i).norm(), tolerance);
  }
  EXPECT_NEAR(new_control.get_damping_eigenvalues().norm(), copy.get_damping_eigenvalues().norm(), tolerance);
}

TEST_F(DissipativeImpedanceControllerTest, TestOrthonormalize) {
  Eigen::Matrix3d basis = Eigen::Matrix3d::Random();
  Eigen::Vector3d eigenvector = Eigen::Vector3d::Random();
  // compute the orthonormal basis
  Eigen::Matrix3d orthonormal_basis = Dissipative<CartesianState>::compute_orthonormal_basis(basis,
                                                                                             eigenvector);
  // first column should the normalized eigenvector
  Eigen::Vector3d err = orthonormal_basis.col(0) - eigenvector.normalized();
  for (int i = 0; i < 3; ++i) { EXPECT_NEAR(err(i), 0., tolerance_); }
  // all inner products must be equal to 0
  EXPECT_NEAR(orthonormal_basis.col(0).dot(orthonormal_basis.col(1)), 0., tolerance_);
  EXPECT_NEAR(orthonormal_basis.col(1).dot(orthonormal_basis.col(2)), 0., tolerance_);
  EXPECT_NEAR(orthonormal_basis.col(0).dot(orthonormal_basis.col(2)), 0., tolerance_);
  // all the magnitude should be equal to 1
  EXPECT_NEAR(orthonormal_basis.col(0).norm(), 1., tolerance_);
  EXPECT_NEAR(orthonormal_basis.col(1).norm(), 1., tolerance_);
  EXPECT_NEAR(orthonormal_basis.col(2).norm(), 1., tolerance_);
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeDampingLinear) {
  Eigen::MatrixXd damping;
  Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
  Eigen::VectorXd eigenvector = Eigen::VectorXd::Random(6);
  Eigen::Matrix3d block;
  // case LINEAR
  set_controller_space(ComputationalSpaceType::LINEAR);
  task_controller_.compute_damping(eigenvector);
  damping = task_controller_.get_damping();
  // linear part is identity
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(damping(i, j), identity3(i, j), tolerance_);
    }
  }
  // angular part is 0
  block = damping.block<3, 3>(3, 3);
  EXPECT_NEAR(std::accumulate(block.data(), block.data() + block.size(), 0.0), 0.0, tolerance_);
  // side blocks are 0
  block = damping.block<3, 3>(3, 0);
  EXPECT_NEAR(std::accumulate(block.data(), block.data() + block.size(), 0.0), 0.0, tolerance_);
  block = damping.block<3, 3>(0, 3);
  EXPECT_NEAR(std::accumulate(block.data(), block.data() + block.size(), 0.0), 0.0, tolerance_);
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeDampingAngular) {
  Eigen::MatrixXd damping;
  Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
  Eigen::VectorXd eigenvector = Eigen::VectorXd::Random(6);
  Eigen::Matrix3d block;
  set_controller_space(ComputationalSpaceType::ANGULAR);
  task_controller_.compute_damping(eigenvector);
  damping = task_controller_.get_damping();
  // linear part is 0
  block = damping.block<3, 3>(0, 0);
  EXPECT_NEAR(std::accumulate(block.data(), block.data() + block.size(), 0.0), 0.0, tolerance_);
  // angular part is non null
  for (int i = 3; i < 6; ++i) {
    for (int j = 3; j < 6; ++j) {
      EXPECT_NEAR(damping(i, j), identity3(i - 3, j - 3), tolerance_);
    }
  }
  // side blocks are 0
  block = damping.block<3, 3>(3, 0);
  EXPECT_NEAR(std::accumulate(block.data(), block.data() + block.size(), 0.0), 0.0, tolerance_);
  block = damping.block<3, 3>(0, 3);
  EXPECT_NEAR(std::accumulate(block.data(), block.data() + block.size(), 0.0), 0.0, tolerance_);
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeDampingDecoupledTwist) {
  Eigen::MatrixXd damping;
  Eigen::MatrixXd identity6 = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd eigenvector = Eigen::VectorXd::Random(6);
  // case DECOUPLED_TWIST
  set_controller_space(ComputationalSpaceType::DECOUPLED_TWIST);
  task_controller_.compute_damping(eigenvector);
  damping = task_controller_.get_damping();
  // identity 6
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(damping(i, j), identity6(i, j), tolerance_);
    }
  }
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeDampingFull) {
  Eigen::MatrixXd damping;
  Eigen::MatrixXd identity6 = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd eigenvector = Eigen::VectorXd::Random(6);
  // case TWIST
  set_controller_space(ComputationalSpaceType::FULL);
  task_controller_.compute_damping(eigenvector);
  damping = task_controller_.get_damping();
  // identity 6
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(damping(i, j), identity6(i, j), tolerance_);
    }
  }
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeCommandWithColinearVelocity) {
  set_controller_space(ComputationalSpaceType::LINEAR);
  // set different damping
  double e1 = 10;
  task_controller_.set_damping_eigenvalue(e1, 0);
  // set a desired and feeadback velocity
  CartesianTwist desired_twist("test", Eigen::Vector3d(1, 0, 0));
  CartesianTwist feedback_twist("test", Eigen::Vector3d(1, 1, 0));
  // first compute the command with a 0 feedback
  CartesianWrench command = task_controller_.compute_command(desired_twist, CartesianTwist::Zero("test"));
  EXPECT_NEAR(command.get_force()(0), 10, tolerance_);
  EXPECT_NEAR(command.get_force()(1), 0, tolerance_);
  EXPECT_NEAR(command.get_force()(2), 0, tolerance_);
  // then compute it with respect to the feedback
  command = task_controller_.compute_command(desired_twist, feedback_twist);
  EXPECT_NEAR(command.get_force()(0), 0, tolerance_);
  EXPECT_NEAR(command.get_force()(1), -1, tolerance_);
  EXPECT_NEAR(command.get_force()(2), 0, tolerance_);
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeTaskToJointCommand) {
  // set a desired and feeadback velocity
  CartesianTwist desired_twist("test", Eigen::Vector3d(1, 0, 0));
  CartesianTwist feedback_twist("test", Eigen::Vector3d(1, 1, 0));
  // set a Jacobian matrix
  Jacobian jacobian("test_robot", 3);
  jacobian.set_data(Eigen::MatrixXd::Random(6, 3));
  // check command
  JointTorques command = task_controller_.compute_command(desired_twist, feedback_twist, jacobian);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeJointCommand) {
  // set a desired and feeadback velocity
  JointVelocities desired_velocities("test", Eigen::Vector4d(1, 0, 0, 0));
  JointVelocities feedback_velocities("test", Eigen::Vector4d(1, 1, 0, 0));
  // check command
  JointTorques command = joint_controller_.compute_command(desired_velocities, feedback_velocities);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}