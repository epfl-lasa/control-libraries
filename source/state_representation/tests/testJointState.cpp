#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

TEST(AddTwoState, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
  Eigen::VectorXd pos2 = Eigen::VectorXd::Random(4);

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState j2("test_robot", 4);
  j2.set_positions(pos2);

  StateRepresentation::JointState jsum = j1 + j2;
  for (unsigned int i = 0; i < j1.get_size(); ++i) {
    EXPECT_TRUE(jsum.get_positions()(i) == j1.get_positions()(i) + j2.get_positions()(i));
  }
}

TEST(SubstractTwoState, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
  Eigen::VectorXd pos2 = Eigen::VectorXd::Random(4);

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState j2("test_robot", 4);
  j2.set_positions(pos2);

  StateRepresentation::JointState jdiff = j1 - j2;
  for (unsigned int i = 0; i < j1.get_size(); ++i) {
    EXPECT_TRUE(jdiff.get_positions()(i) == j1.get_positions()(i) - j2.get_positions()(i));
  }
}

TEST(MultiplyByScalar, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState jsum = 0.5 * j1;
  for (unsigned int i = 0; i < j1.get_size(); ++i) {
    EXPECT_TRUE(jsum.get_positions()(i) == 0.5 * j1.get_positions()(i));
  }
}

TEST(MultiplyByArray, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState jsum = gain * j1;
  for (unsigned int i = 0; i < j1.get_size(); ++i) {
    EXPECT_TRUE(jsum.get_positions()(i) == gain(i, i) * j1.get_positions()(i));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}