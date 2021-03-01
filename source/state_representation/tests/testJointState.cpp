#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

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

TEST(GetData, PositiveNos) {
  StateRepresentation::JointState js = StateRepresentation::JointState::Random("test_robot", 4);
  Eigen::VectorXd concatenated_state(js.get_size() * 4);
  concatenated_state << js.get_positions(), js.get_velocities(), js.get_accelerations(), js.get_torques();
  EXPECT_NEAR(concatenated_state.norm(), js.data().norm(), 1e-4);
}

TEST(JointStateToStdVector, PositiveNos) {
  StateRepresentation::JointState js = StateRepresentation::JointState::Random("test_robot", 4);
  std::vector<double> vec_data = js.to_std_vector();
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(js.data()(i) == vec_data[i]);
  }
}

TEST(JointPositionsToStdVector, PositiveNos) {
  StateRepresentation::JointPositions jp = StateRepresentation::JointPositions::Random("test_robot", 4);
  std::vector<double> vec_data = jp.to_std_vector();
  EXPECT_TRUE(vec_data.size() == jp.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(jp.get_positions()(i) == vec_data[i]);
  }
}

TEST(JointVelocitiesToStdVector, PositiveNos) {
  StateRepresentation::JointVelocities jv = StateRepresentation::JointVelocities::Random("test_robot", 4);
  std::vector<double> vec_data = jv.to_std_vector();
  EXPECT_TRUE(vec_data.size() == jv.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(jv.get_velocities()(i) == vec_data[i]);
  }
}

TEST(JointTorquesToStdVector, PositiveNos) {
  StateRepresentation::JointTorques jt = StateRepresentation::JointTorques::Random("test_robot", 4);
  std::vector<double> vec_data = jt.to_std_vector();
  EXPECT_TRUE(vec_data.size() == jt.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(jt.get_torques()(i) == vec_data[i]);
  }
}

TEST(AddTwoState, PositiveNos) {
  StateRepresentation::JointState j1 = StateRepresentation::JointState::Random("test_robot", 4);
  StateRepresentation::JointState j2 = StateRepresentation::JointState::Random("test_robot", 4);
  StateRepresentation::JointState jsum = j1 + j2;
  EXPECT_NEAR(jsum.data().norm(), (j1.data() + j2.data()).norm(), 1e-4);
}

TEST(SubstractTwoState, PositiveNos) {
  StateRepresentation::JointState j1 = StateRepresentation::JointState::Random("test_robot", 4);
  StateRepresentation::JointState j2 = StateRepresentation::JointState::Random("test_robot", 4);
  StateRepresentation::JointState jdiff = j1 - j2;
  EXPECT_NEAR(jdiff.data().norm(), (j1.data() - j2.data()).norm(), 1e-4);
}

TEST(MultiplyByScalar, PositiveNos) {
  StateRepresentation::JointState js = StateRepresentation::JointState::Random("test_robot", 4);
  StateRepresentation::JointState jscaled = 0.5 * js;
  EXPECT_NEAR(jscaled.data().norm(), (0.5 * js.data()).norm(), 1e-4);
}

TEST(MultiplyByArray, PositiveNos) {
  StateRepresentation::JointState js = StateRepresentation::JointState::Random("test_robot", 4);
  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();
  StateRepresentation::JointState jscaled = gain * js;
  EXPECT_NEAR(jscaled.data().norm(), (gain * js.data()).norm(), 1e-4);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}