#include <gtest/gtest.h>

#include "controllers/ControllerFactory.hpp"
#include "controllers/impedance/Dissipative.hpp"

#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include "state_representation/space/joint/JointTorques.hpp"

using namespace controllers;
using namespace controllers::impedance;
using namespace state_representation;

class DissipativeWrapper : public Dissipative<CartesianState> {
public:
  DissipativeWrapper(const ComputationalSpaceType& computational_space) :
      Dissipative<CartesianState>(computational_space) {}

  static Eigen::MatrixXd
  wrap_orthonormalize_basis(const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector) {
    return Dissipative<CartesianState>::orthonormalize_basis(basis, main_eigenvector);
  }

  Eigen::MatrixXd wrap_compute_orthonormal_basis(const CartesianState& desired_velocity) {
    return this->compute_orthonormal_basis(desired_velocity);
  }

  void wrap_compute_damping(const CartesianState& desired_velocity) {
    return this->compute_damping(desired_velocity);
  }
};

class DissipativeControllerMethodTest : public testing::Test {
protected:
  void SetUp() override {
    set_controller_space(ComputationalSpaceType::FULL);
  }

  void set_controller_space(const ComputationalSpaceType& computational_space) {
    controller_ = std::make_shared<DissipativeWrapper>(computational_space);
  }

  std::shared_ptr<DissipativeWrapper> controller_;
  double tolerance_ = 1e-4;
};

TEST_F(DissipativeControllerMethodTest, TestOrthonormalize) {
  Eigen::Matrix3d basis = Eigen::Matrix3d::Random();
  Eigen::Vector3d eigenvector = Eigen::Vector3d::Random();
  // compute the orthonormal basis
  Eigen::Matrix3d orthonormal_basis = controller_->wrap_orthonormalize_basis(basis, eigenvector);
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

TEST_F(DissipativeControllerMethodTest, TestComputeDampingLinear) {
  Eigen::MatrixXd damping;
  CartesianTwist vel = CartesianTwist::Random("test");
  Eigen::Matrix3d block;
  set_controller_space(ComputationalSpaceType::LINEAR);
  controller_->wrap_compute_damping(vel);
  damping = controller_->get_parameter_value<Eigen::MatrixXd>("damping");
  // linear part is identity
  block = damping.topLeftCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Identity()));
  // angular part is 0
  block = damping.bottomRightCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Zero()));
  // side blocks are 0
  block = damping.topRightCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Zero()));
  block = damping.bottomLeftCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Zero()));
}

TEST_F(DissipativeControllerMethodTest, TestComputeDampingAngular) {
  Eigen::MatrixXd damping;
  CartesianTwist vel = CartesianTwist::Random("test");
  Eigen::Matrix3d block;
  set_controller_space(ComputationalSpaceType::ANGULAR);
  controller_->wrap_compute_damping(vel);
  damping = controller_->get_parameter_value<Eigen::MatrixXd>("damping");
  // linear part is 0
  block = damping.topLeftCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Zero()));
  // angular part is identity
  block = damping.bottomRightCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Identity()));
  // side blocks are 0
  block = damping.topRightCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Zero()));
  block = damping.bottomLeftCorner<3, 3>();
  EXPECT_TRUE(block.isApprox(Eigen::Matrix3d::Zero()));
}

TEST_F(DissipativeControllerMethodTest, TestComputeDampingDecoupledTwist) {
  Eigen::MatrixXd damping;
  CartesianTwist vel = CartesianTwist::Random("test");
  set_controller_space(ComputationalSpaceType::DECOUPLED_TWIST);
  controller_->wrap_compute_damping(vel);
  damping = controller_->get_parameter_value<Eigen::MatrixXd>("damping");
  EXPECT_TRUE(damping.isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

TEST_F(DissipativeControllerMethodTest, TestComputeDampingFull) {
  Eigen::MatrixXd damping;
  CartesianTwist vel = CartesianTwist::Random("test");
  set_controller_space(ComputationalSpaceType::FULL);
  controller_->wrap_compute_damping(vel);
  damping = controller_->get_parameter_value<Eigen::MatrixXd>("damping");
  EXPECT_TRUE(damping.isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisZeroLinear) {
  CartesianTwist vel = CartesianTwist::Zero("test");
  set_controller_space(ComputationalSpaceType::LINEAR);
  // then compute it
  Eigen::MatrixXd basis = controller_->wrap_compute_orthonormal_basis(vel);
  Eigen::Matrix3d linear_block = basis.topLeftCorner<3, 3>();
  Eigen::Matrix3d angular_block = basis.bottomRightCorner<3, 3>();
  EXPECT_TRUE(linear_block.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(angular_block.isApprox(Eigen::Matrix3d::Zero()));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisZeroAngular) {
  CartesianTwist vel = CartesianTwist::Zero("test");
  set_controller_space(ComputationalSpaceType::ANGULAR);
  // then compute it
  Eigen::MatrixXd basis = controller_->wrap_compute_orthonormal_basis(vel);
  Eigen::Matrix3d linear_block = basis.topLeftCorner<3, 3>();
  Eigen::Matrix3d angular_block = basis.bottomRightCorner<3, 3>();
  EXPECT_TRUE(linear_block.isApprox(Eigen::Matrix3d::Zero()));
  EXPECT_TRUE(angular_block.isApprox(Eigen::Matrix3d::Identity()));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisZeroDecoupledTwist) {
  CartesianTwist vel = CartesianTwist::Zero("test");
  set_controller_space(ComputationalSpaceType::DECOUPLED_TWIST);
  // then compute it
  Eigen::MatrixXd basis = controller_->wrap_compute_orthonormal_basis(vel);
  EXPECT_TRUE(basis.isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisNonZeroLinearDecoupledTwist) {
  CartesianTwist vel = CartesianTwist::Zero("test");
  vel.set_linear_velocity(Eigen::Vector3d::Random());
  set_controller_space(ComputationalSpaceType::DECOUPLED_TWIST);
  // then compute it
  Eigen::MatrixXd basis = controller_->wrap_compute_orthonormal_basis(vel);
  EXPECT_FALSE(basis.isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisNonZeroAngularDecoupledTwist) {
  CartesianTwist vel = CartesianTwist::Zero("test");
  vel.set_angular_velocity(Eigen::Vector3d::Random());
  set_controller_space(ComputationalSpaceType::DECOUPLED_TWIST);
  // then compute it
  Eigen::MatrixXd basis = controller_->wrap_compute_orthonormal_basis(vel);
  EXPECT_FALSE(basis.isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisZeroFull) {
  CartesianTwist vel = CartesianTwist::Zero("test");
  set_controller_space(ComputationalSpaceType::FULL);
  Eigen::MatrixXd basis = controller_->wrap_compute_orthonormal_basis(vel);
  EXPECT_TRUE(basis.isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

/*
TEST_F(DissipativeControllerMethodTest, TestComputeBasisJointState) {
  JointVelocities vel = JointVelocities::Random("test", 4);
  Eigen::MatrixXd basis = joint_controller_.compute_orthonormal_basis(vel);
  EXPECT_FALSE(basis.isApprox(Eigen::MatrixXd::Identity(4, 4)));
}

TEST_F(DissipativeControllerMethodTest, TestComputeBasisZeroJointState) {
  JointVelocities vel = JointVelocities::Zero("test", 4);
  Eigen::MatrixXd basis = joint_controller_.compute_orthonormal_basis(vel);
  EXPECT_TRUE(basis.isApprox(Eigen::MatrixXd::Identity(4, 4)));
}
*/

TEST(DissipativeControllerTest, TestComputeCommandWithColinearVelocity) {
  double tolerance = 1e-4;
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE_LINEAR);

  // set different damping
  auto eigenvalues = controller->get_parameter_value<Eigen::VectorXd>("damping_eigenvalues");
  eigenvalues(0) = 10.0;
  controller->set_parameter_value("damping_eigenvalues", eigenvalues);

  // set a desired and feeadback velocity
  CartesianTwist desired_twist("test", Eigen::Vector3d(1, 0, 0));
  CartesianTwist feedback_twist("test", Eigen::Vector3d(1, 1, 0));
  // first compute the command with a 0 feedback
  CartesianWrench command = controller->compute_command(desired_twist, CartesianTwist::Zero("test"));
  EXPECT_NEAR(command.get_force()(0), 10, tolerance);
  EXPECT_NEAR(command.get_force()(1), 0, tolerance);
  EXPECT_NEAR(command.get_force()(2), 0, tolerance);
  // then compute it with respect to the feedback
  command = controller->compute_command(desired_twist, feedback_twist);
  EXPECT_NEAR(command.get_force()(0), 0, tolerance);
  EXPECT_NEAR(command.get_force()(1), -1, tolerance);
  EXPECT_NEAR(command.get_force()(2), 0, tolerance);
}

TEST(DissipativeControllerTest, TestComputeJointCommand) {
  auto controller = JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE, 4);
  // set a desired and feeadback velocity
  JointVelocities desired_velocities("test", Eigen::Vector4d(1, 0, 0, 0));
  JointVelocities feedback_velocities("test", Eigen::Vector4d(1, 1, 0, 0));
  // check command
  JointTorques command = controller->compute_command(desired_velocities, feedback_velocities);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}

TEST(DissipativeControllerTest, TestComputeTaskToJointCommand) {
  auto controller = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE);
  // set a desired and feeadback velocity
  CartesianTwist desired_twist("test", Eigen::Vector3d(1, 0, 0));
  CartesianTwist feedback_twist("test", Eigen::Vector3d(1, 1, 0));
  // set a Jacobian matrix
  Jacobian jac = Jacobian::Random("test_robot", 3, "test");
  // check command
  JointTorques command = controller->compute_command(desired_twist, feedback_twist, jac);
  // expect some non null data
  EXPECT_TRUE(command.data().norm() > 0.);
}
