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

TEST(ZeroInitialization, PositiveNos) {
  StateRepresentation::JointState zero = StateRepresentation::JointState::Zero("test", 3);
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero.is_empty());
  // all data should be zero
  EXPECT_TRUE(zero.data().norm() == 0);
  // same for the second initializer
  StateRepresentation::JointState zero2 = StateRepresentation::JointState::Zero("test",
                                                                                std::vector<std::string>{"j0", "j1"});
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero2.is_empty());
  // all data should be zero
  EXPECT_TRUE(zero2.data().norm() == 0);
}

TEST(RandomStateInitialization, PositiveNos) {
  StateRepresentation::JointState random = StateRepresentation::JointState::Random("test", 3);
  // all data should be random (non 0)
  EXPECT_TRUE(random.get_positions().norm() > 0);
  EXPECT_TRUE(random.get_velocities().norm() > 0);
  EXPECT_TRUE(random.get_accelerations().norm() > 0);
  EXPECT_TRUE(random.get_torques().norm() > 0);
  // same for the second initializer
  StateRepresentation::JointState random2 =
      StateRepresentation::JointState::Random("test",
                                              std::vector<std::string>{"j0", "j1"});
  // all data should be random (non 0)
  EXPECT_TRUE(random2.get_positions().norm() > 0);
  EXPECT_TRUE(random2.get_velocities().norm() > 0);
  EXPECT_TRUE(random2.get_accelerations().norm() > 0);
  EXPECT_TRUE(random2.get_torques().norm() > 0);
}

TEST(RandomPositionsInitialization, PositiveNos) {
  StateRepresentation::JointPositions random = StateRepresentation::JointPositions::Random("test", 3);
  // only position should be random
  EXPECT_TRUE(random.get_positions().norm() > 0);
  EXPECT_TRUE(random.get_velocities().norm() == 0);
  EXPECT_TRUE(random.get_accelerations().norm() == 0);
  EXPECT_TRUE(random.get_torques().norm() == 0);
  // same for the second initializer
  StateRepresentation::JointPositions random2 =
      StateRepresentation::JointPositions::Random("test",
                                              std::vector<std::string>{"j0", "j1"});
  // only position should be random
  EXPECT_TRUE(random2.get_positions().norm() > 0);
  EXPECT_TRUE(random2.get_velocities().norm() == 0);
  EXPECT_TRUE(random2.get_accelerations().norm() == 0);
  EXPECT_TRUE(random2.get_torques().norm() == 0);
}

TEST(RandomVelocitiesInitialization, PositiveNos) {
  StateRepresentation::JointVelocities random = StateRepresentation::JointVelocities::Random("test", 3);
  // only velocities should be random
  EXPECT_TRUE(random.get_positions().norm() == 0);
  EXPECT_TRUE(random.get_velocities().norm() > 0);
  EXPECT_TRUE(random.get_accelerations().norm() == 0);
  EXPECT_TRUE(random.get_torques().norm() == 0);
  // same for the second initializer
  StateRepresentation::JointVelocities random2 =
      StateRepresentation::JointVelocities::Random("test",
                                                  std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_TRUE(random2.get_positions().norm() == 0);
  EXPECT_TRUE(random2.get_velocities().norm() > 0);
  EXPECT_TRUE(random2.get_accelerations().norm() == 0);
  EXPECT_TRUE(random2.get_torques().norm() == 0);
}

TEST(RandomTorquesInitialization, PositiveNos) {
  StateRepresentation::JointTorques random = StateRepresentation::JointTorques::Random("test", 3);
  // only torques should be random
  EXPECT_TRUE(random.get_positions().norm() == 0);
  EXPECT_TRUE(random.get_velocities().norm() == 0);
  EXPECT_TRUE(random.get_accelerations().norm() == 0);
  EXPECT_TRUE(random.get_torques().norm() > 0);
  // same for the second initializer
  StateRepresentation::JointTorques random2 =
      StateRepresentation::JointTorques::Random("test",
                                                   std::vector<std::string>{"j0", "j1"});
  // only torques should be random
  EXPECT_TRUE(random2.get_positions().norm() == 0);
  EXPECT_TRUE(random2.get_velocities().norm() == 0);
  EXPECT_TRUE(random2.get_accelerations().norm() == 0);
  EXPECT_TRUE(random2.get_torques().norm() > 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}