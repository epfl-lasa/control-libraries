#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

TEST(SetPositonsWithModulo, PositiveNos) {
  Eigen::VectorXd positions(4);
  positions << 1, 2, 3, 4;

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(positions);

  std::cerr << j1 << std::endl;

  for (unsigned int i = 0; i < j1.get_size(); ++i) EXPECT_TRUE(-M_PI < j1.get_positions()(i) && j1.get_positions()(i) < M_PI);
}

TEST(SetPositonsWithModuloAndNegativeNumbers, PositiveNos) {
  Eigen::VectorXd positions(4);
  positions << -1, -2, -3, -4;

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(positions);

  std::cerr << j1 << std::endl;

  for (unsigned int i = 0; i < j1.get_size(); ++i) EXPECT_TRUE(-M_PI < j1.get_positions()(i) && j1.get_positions()(i) < M_PI);
  EXPECT_TRUE(j1.get_positions()(0) < 0 && j1.get_positions()(3) > 0);
}

TEST(AddTwoState, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
  Eigen::VectorXd pos2 = Eigen::VectorXd::Random(4);

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState j2("test_robot", 4);
  j2.set_positions(pos2);

  StateRepresentation::JointState jsum = j1 + j2;
  for (unsigned int i = 0; i < j1.get_size(); ++i) EXPECT_TRUE(jsum.get_positions()(i) == atan2(sin(j1.get_positions()(i) + j2.get_positions()(i)), cos(j1.get_positions()(i) + j2.get_positions()(i))));
}

TEST(MultiplyByScalar, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState jsum = 0.5 * j1;
  for (unsigned int i = 0; i < j1.get_size(); ++i) EXPECT_TRUE(jsum.get_positions()(i) == atan2(sin(0.5 * j1.get_positions()(i)), cos(0.5 * j1.get_positions()(i))));
}

TEST(MultiplyByArray, PositiveNos) {
  Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();

  StateRepresentation::JointState j1("test_robot", 4);
  j1.set_positions(pos1);

  StateRepresentation::JointState jsum = gain * j1;
  for (unsigned int i = 0; i < j1.get_size(); ++i) {
    EXPECT_TRUE(jsum.get_positions()(i) == atan2(sin(gain(i, i) * j1.get_positions()(i)), cos(gain(i, i) * j1.get_positions()(i))));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}