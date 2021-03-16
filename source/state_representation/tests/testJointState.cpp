#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

TEST(ZeroInitialization, PositiveNos) {
  state_representation::JointState zero = state_representation::JointState::Zero("test", 3);
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero.is_empty());
  // all data should be zero
  EXPECT_TRUE(zero.data().norm() == 0);
  // same for the second initializer
  state_representation::JointState zero2 = state_representation::JointState::Zero("test",
                                                                                  std::vector<std::string>{"j0", "j1"});
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero2.is_empty());
  // all data should be zero
  EXPECT_TRUE(zero2.data().norm() == 0);
}

TEST(RandomStateInitialization, PositiveNos) {
  state_representation::JointState random = state_representation::JointState::Random("test", 3);
  // all data should be random (non 0)
  EXPECT_TRUE(random.get_positions().norm() > 0);
  EXPECT_TRUE(random.get_velocities().norm() > 0);
  EXPECT_TRUE(random.get_accelerations().norm() > 0);
  EXPECT_TRUE(random.get_torques().norm() > 0);
  // same for the second initializer
  state_representation::JointState random2 =
      state_representation::JointState::Random("test",
                                               std::vector<std::string>{"j0", "j1"});
  // all data should be random (non 0)
  EXPECT_TRUE(random2.get_positions().norm() > 0);
  EXPECT_TRUE(random2.get_velocities().norm() > 0);
  EXPECT_TRUE(random2.get_accelerations().norm() > 0);
  EXPECT_TRUE(random2.get_torques().norm() > 0);
}

TEST(RandomPositionsInitialization, PositiveNos) {
  state_representation::JointPositions random = state_representation::JointPositions::Random("test", 3);
  // only position should be random
  EXPECT_TRUE(random.get_positions().norm() > 0);
  EXPECT_TRUE(random.get_velocities().norm() == 0);
  EXPECT_TRUE(random.get_accelerations().norm() == 0);
  EXPECT_TRUE(random.get_torques().norm() == 0);
  // same for the second initializer
  state_representation::JointPositions random2 =
      state_representation::JointPositions::Random("test",
                                                   std::vector<std::string>{"j0", "j1"});
  // only position should be random
  EXPECT_TRUE(random2.get_positions().norm() > 0);
  EXPECT_TRUE(random2.get_velocities().norm() == 0);
  EXPECT_TRUE(random2.get_accelerations().norm() == 0);
  EXPECT_TRUE(random2.get_torques().norm() == 0);
}

TEST(RandomVelocitiesInitialization, PositiveNos) {
  state_representation::JointVelocities random = state_representation::JointVelocities::Random("test", 3);
  // only velocities should be random
  EXPECT_TRUE(random.get_positions().norm() == 0);
  EXPECT_TRUE(random.get_velocities().norm() > 0);
  EXPECT_TRUE(random.get_accelerations().norm() == 0);
  EXPECT_TRUE(random.get_torques().norm() == 0);
  // same for the second initializer
  state_representation::JointVelocities random2 =
      state_representation::JointVelocities::Random("test",
                                                    std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_TRUE(random2.get_positions().norm() == 0);
  EXPECT_TRUE(random2.get_velocities().norm() > 0);
  EXPECT_TRUE(random2.get_accelerations().norm() == 0);
  EXPECT_TRUE(random2.get_torques().norm() == 0);
}

TEST(RandomTorquesInitialization, PositiveNos) {
  state_representation::JointTorques random = state_representation::JointTorques::Random("test", 3);
  // only torques should be random
  EXPECT_TRUE(random.get_positions().norm() == 0);
  EXPECT_TRUE(random.get_velocities().norm() == 0);
  EXPECT_TRUE(random.get_accelerations().norm() == 0);
  EXPECT_TRUE(random.get_torques().norm() > 0);
  // same for the second initializer
  state_representation::JointTorques random2 =
      state_representation::JointTorques::Random("test",
                                                 std::vector<std::string>{"j0", "j1"});
  // only torques should be random
  EXPECT_TRUE(random2.get_positions().norm() == 0);
  EXPECT_TRUE(random2.get_velocities().norm() == 0);
  EXPECT_TRUE(random2.get_accelerations().norm() == 0);
  EXPECT_TRUE(random2.get_torques().norm() > 0);
}

TEST(GetData, PositiveNos) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  Eigen::VectorXd concatenated_state(js.get_size() * 4);
  concatenated_state << js.get_positions(), js.get_velocities(), js.get_accelerations(), js.get_torques();
  EXPECT_NEAR(concatenated_state.norm(), js.data().norm(), 1e-4);
}

TEST(JointStateToStdVector, PositiveNos) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  std::vector<double> vec_data = js.to_std_vector();
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(js.data()(i) == vec_data[i]);
  }
}

TEST(JointPositionsToStdVector, PositiveNos) {
  state_representation::JointPositions jp = state_representation::JointPositions::Random("test_robot", 4);
  std::vector<double> vec_data = jp.to_std_vector();
  EXPECT_TRUE(vec_data.size() == jp.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(jp.get_positions()(i) == vec_data[i]);
  }
}

TEST(JointVelocitiesToStdVector, PositiveNos) {
  state_representation::JointVelocities jv = state_representation::JointVelocities::Random("test_robot", 4);
  std::vector<double> vec_data = jv.to_std_vector();
  EXPECT_TRUE(vec_data.size() == jv.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(jv.get_velocities()(i) == vec_data[i]);
  }
}

TEST(JointTorquesToStdVector, PositiveNos) {
  state_representation::JointTorques jt = state_representation::JointTorques::Random("test_robot", 4);
  std::vector<double> vec_data = jt.to_std_vector();
  EXPECT_TRUE(vec_data.size() == jt.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_TRUE(jt.get_torques()(i) == vec_data[i]);
  }
}

TEST(AddTwoState, PositiveNos) {
  state_representation::JointState j1 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState j2 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState jsum = j1 + j2;
  EXPECT_NEAR(jsum.data().norm(), (j1.data() + j2.data()).norm(), 1e-4);
}

TEST(SubstractTwoState, PositiveNos) {
  state_representation::JointState j1 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState j2 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState jdiff = j1 - j2;
  EXPECT_NEAR(jdiff.data().norm(), (j1.data() - j2.data()).norm(), 1e-4);
}

TEST(MultiplyByScalar, PositiveNos) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState jscaled = 0.5 * js;
  EXPECT_NEAR(jscaled.data().norm(), (0.5 * js.data()).norm(), 1e-4);
}

TEST(MultiplyByArray, PositiveNos) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();
  state_representation::JointState jscaled = gain * js;
  EXPECT_NEAR(jscaled.data().norm(), (gain * js.data()).norm(), 1e-4);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}