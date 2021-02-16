#include "controllers/impedance/Dissipative.hpp"
#include <gtest/gtest.h>

using namespace controllers::impedance;
using namespace StateRepresentation;

class DissipativeImpedanceControllerTest : public testing::Test {
protected:
  void SetUp() override {}

  void set_controller_space(const ComputationalSpaceType& computational_space) {
    controller_ = Dissipative(computational_space);
  }

  Dissipative controller_;
};

TEST_F(DissipativeImpedanceControllerTest, TestOrthonormalize) {
  Eigen::Matrix3d basis = Eigen::Matrix3d::Random();
  Eigen::Vector3d eigenvector = Eigen::Vector3d::Random();

  // compute the orthonormal basis
  Eigen::Matrix3d orthonormal_basis = controller_.compute_orthonormal_basis(basis, eigenvector);

  // first column should the normalized eigenvector
  Eigen::Vector3d err = orthonormal_basis.col(0) - eigenvector.normalized();
  for (int i = 0; i < 3; ++i) { EXPECT_NEAR(err(i), 0., 10e-4); }
  // all inner products must be equal to 0
  EXPECT_NEAR(orthonormal_basis.col(0).dot(orthonormal_basis.col(1)), 0., 10e-4);
  EXPECT_NEAR(orthonormal_basis.col(1).dot(orthonormal_basis.col(2)), 0., 10e-4);
  EXPECT_NEAR(orthonormal_basis.col(0).dot(orthonormal_basis.col(2)), 0., 10e-4);
  // all the magnitude should be equal to 1
  EXPECT_NEAR(orthonormal_basis.col(0).norm(), 1., 10e-4);
  EXPECT_NEAR(orthonormal_basis.col(1).norm(), 1., 10e-4);
  EXPECT_NEAR(orthonormal_basis.col(2).norm(), 1., 10e-4);
}

TEST_F(DissipativeImpedanceControllerTest, TestComputeDamping) {
  Eigen::MatrixXd damping;
  Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd identity6 = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd eigenvector = Eigen::VectorXd::Random(6);
  // case LINEAR
  set_controller_space(ComputationalSpaceType::LINEAR);
  controller_.compute_damping(eigenvector);
  damping = controller_.get_damping();

  // linear part is identity
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) { EXPECT_NEAR(damping(i, j), identity3(i, j), 10e-4); }
  }
  // angular part is 0
  for (int i = 3; i < 6; ++i) {
    for (int j = 3; j < 6; ++j) { EXPECT_NEAR(damping(i, j), 0., 10e-4); }
  }
  // side blocks are 0
  for (int i = 3; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) { EXPECT_NEAR(damping(i, j), 0., 10e-4); }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 3; j < 6; ++j) { EXPECT_NEAR(damping(i, j), 0., 10e-4); }
  }

  // case ANGULAR
  set_controller_space(ComputationalSpaceType::ANGULAR);
  controller_.compute_damping(eigenvector);
  damping = controller_.get_damping();
  // linear part is 0
  for (int i = 0; i < 3; ++i) {
    for (int j = 3; j < 3; ++j) { EXPECT_NEAR(damping(i, j), 0., 10e-4); }
  }
  // angular part is non null
  for (int i = 3; i < 6; ++i) {
    for (int j = 3; j < 6; ++j) { EXPECT_NEAR(damping(i, j), identity3(i - 3, j - 3), 10e-4); }
  }
  // side blocks are 0
  for (int i = 3; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) { EXPECT_NEAR(damping(i, j), 0., 10e-4); }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 3; j < 6; ++j) { EXPECT_NEAR(damping(i, j), 0., 10e-4); }
  }

  // case DECOUPLED_TWIST
  set_controller_space(ComputationalSpaceType::DECOUPLED_TWIST);
  controller_.compute_damping(eigenvector);
  damping = controller_.get_damping();
  // identity 6
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) { EXPECT_NEAR(damping(i, j), identity6(i, j), 10e-4); }
  }

  // case TWIST
  set_controller_space(ComputationalSpaceType::TWIST);
  controller_.compute_damping(eigenvector);
  damping = controller_.get_damping();
  // identity 6
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) { EXPECT_NEAR(damping(i, j), identity6(i, j), 10e-4); }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}