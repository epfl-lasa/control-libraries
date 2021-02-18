#include "state_representation/Robot/Jacobian.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

TEST(TestCreate, PositiveNos) {
  StateRepresentation::Jacobian jac("robot", Eigen::MatrixXd::Random(6, 7));

  EXPECT_TRUE(jac.get_nb_rows() == 6);
  EXPECT_TRUE(jac.get_nb_cols() == 7);

  bool except_thrown = false;
  try {
    jac.set_data(Eigen::MatrixXd::Random(7, 6));
  } catch (const IncompatibleSizeException& e) {
    except_thrown = true;
  }
  EXPECT_TRUE(except_thrown);
}

TEST(TestTranspose, PositiveNos) {
  StateRepresentation::Jacobian jac("robot", Eigen::MatrixXd::Random(6, 7));
  jac = jac.transpose();

  EXPECT_TRUE(jac.get_nb_rows() == 7);
  EXPECT_TRUE(jac.get_nb_cols() == 6);

  bool except_thrown = false;
  try {
    jac.set_data(Eigen::MatrixXd::Random(7, 6));
  } catch (const IncompatibleSizeException& e) {
    except_thrown = true;
  }
  EXPECT_FALSE(except_thrown);
}

TEST(TestMutltiplyWithEigen, PositiveNos) {
  StateRepresentation::Jacobian jac("robot", Eigen::MatrixXd::Random(6, 7));
  Eigen::MatrixXd mat1 = Eigen::VectorXd::Random(7, 1);
  Eigen::MatrixXd res1 = jac * mat1;

  EXPECT_TRUE(res1.rows() == 6);
  EXPECT_TRUE(res1.cols() == 1);

  Eigen::MatrixXd mat2 = Eigen::VectorXd::Random(6, 1);
  bool except_thrown = false;
  try {
    Eigen::MatrixXd res2 = jac * mat2;
  } catch (const IncompatibleSizeException& e) {
    except_thrown = true;
  }
  EXPECT_TRUE(except_thrown);

  Eigen::MatrixXd res2 = jac.transpose() * mat2;
  EXPECT_TRUE(res2.rows() == 7);
  EXPECT_TRUE(res2.cols() == 1);
}

TEST(TestSolve, PositiveNos) {
  StateRepresentation::Jacobian jac("robot", Eigen::MatrixXd::Random(6, 7));
  Eigen::MatrixXd mat1 = Eigen::VectorXd::Random(7, 1);
  bool except_thrown = false;
  try {
    Eigen::MatrixXd res1 = jac.solve(mat1);
  } catch (const IncompatibleSizeException& e) {
    except_thrown = true;
  }
  EXPECT_TRUE(except_thrown);

  Eigen::MatrixXd mat2 = Eigen::VectorXd::Random(6, 1);
  Eigen::MatrixXd res2 = jac.solve(mat2);

  EXPECT_TRUE(res2.rows() == 7);
  EXPECT_TRUE(res2.cols() == 1);
}

TEST(TestJointToCartesian, PositiveNos) {
  StateRepresentation::Jacobian jac("robot", Eigen::MatrixXd::Random(6, 7));
  StateRepresentation::JointVelocities jvel("robot", Eigen::VectorXd::Random(7));

  bool except_thrown = false;
  try {
    StateRepresentation::CartesianTwist cvel = jac * jvel;
  } catch (const IncompatibleSizeException& e) {
    except_thrown = true;
  }
  EXPECT_FALSE(except_thrown);
}

TEST(TestCartesianToJoint, PositiveNos) {
  StateRepresentation::Jacobian jac("robot", Eigen::MatrixXd::Random(6, 7));
  Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Random();
  StateRepresentation::CartesianTwist cvel("robot", vec);

  StateRepresentation::JointVelocities jvel1;
  bool except_thrown1 = false;
  try {
    jvel1 = jac.solve(cvel);
  } catch (const IncompatibleSizeException& e) {
    except_thrown1 = true;
  }
  EXPECT_FALSE(except_thrown1);

  StateRepresentation::JointVelocities jvel2;
  bool except_thrown2 = false;
  try {
    jvel2 = jac.pseudoinverse() * cvel;
  } catch (const IncompatibleSizeException& e) {
    except_thrown2 = true;
  }
  EXPECT_FALSE(except_thrown2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}