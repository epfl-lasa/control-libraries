#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

TEST(JointStateTest, ZeroInitialization) {
  state_representation::JointState zero = state_representation::JointState::Zero("test", 3);
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero.is_empty());
  // all data should be zero
  EXPECT_EQ(zero.data().norm(), 0);
  // same for the second initializer
  state_representation::JointState zero2 = state_representation::JointState::Zero("test",
                                                                                  std::vector<std::string>{"j0", "j1"});
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero2.is_empty());
  // all data should be zero
  EXPECT_EQ(zero2.data().norm(), 0);
}

TEST(JointStateTest, RandomStateInitialization) {
  state_representation::JointState random = state_representation::JointState::Random("test", 3);
  // all data should be random (non 0)
  EXPECT_GT(random.get_positions().norm(), 0);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_GT(random.get_accelerations().norm(), 0);
  EXPECT_GT(random.get_torques().norm(), 0);
  // same for the second initializer
  state_representation::JointState random2 =
      state_representation::JointState::Random("test",
                                               std::vector<std::string>{"j0", "j1"});
  // all data should be random (non 0)
  EXPECT_GT(random2.get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_GT(random2.get_accelerations().norm(), 0);
  EXPECT_GT(random2.get_torques().norm(), 0);
}

TEST(JointStateTest, RandomPositionsInitialization) {
  state_representation::JointPositions random = state_representation::JointPositions::Random("test", 3);
  // only position should be random
  EXPECT_GT(random.get_positions().norm(), 0);
  EXPECT_EQ(random.get_velocities().norm(), 0);
  EXPECT_EQ(random.get_accelerations().norm(), 0);
  EXPECT_EQ(random.get_torques().norm(), 0);
  // same for the second initializer
  state_representation::JointPositions random2 =
      state_representation::JointPositions::Random("test",
                                                   std::vector<std::string>{"j0", "j1"});
  // only position should be random
  EXPECT_GT(random2.get_positions().norm(), 0);
  EXPECT_EQ(random2.get_velocities().norm(), 0);
  EXPECT_EQ(random2.get_accelerations().norm(), 0);
  EXPECT_EQ(random2.get_torques().norm(), 0);
}

TEST(JointStateTest, RandomVelocitiesInitialization) {
  state_representation::JointVelocities random = state_representation::JointVelocities::Random("test", 3);
  // only velocities should be random
  EXPECT_EQ(random.get_positions().norm(), 0);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_EQ(random.get_accelerations().norm(), 0);
  EXPECT_EQ(random.get_torques().norm(), 0);
  // same for the second initializer
  state_representation::JointVelocities random2 =
      state_representation::JointVelocities::Random("test",
                                                    std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_EQ(random2.get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_EQ(random2.get_accelerations().norm(), 0);
  EXPECT_EQ(random2.get_torques().norm(), 0);
}

TEST(JointStateTest, RandomTorquesInitialization) {
  state_representation::JointTorques random = state_representation::JointTorques::Random("test", 3);
  // only torques should be random
  EXPECT_EQ(random.get_positions().norm(), 0);
  EXPECT_EQ(random.get_velocities().norm(), 0);
  EXPECT_EQ(random.get_accelerations().norm(), 0);
  EXPECT_GT(random.get_torques().norm(), 0);
  // same for the second initializer
  state_representation::JointTorques random2 =
      state_representation::JointTorques::Random("test",
                                                 std::vector<std::string>{"j0", "j1"});
  // only torques should be random
  EXPECT_EQ(random2.get_positions().norm(), 0);
  EXPECT_EQ(random2.get_velocities().norm(), 0);
  EXPECT_EQ(random2.get_accelerations().norm(), 0);
  EXPECT_GT(random2.get_torques().norm(), 0);
}

TEST(JointStateTest, GetData) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  Eigen::VectorXd concatenated_state(js.get_size() * 4);
  concatenated_state << js.get_positions(), js.get_velocities(), js.get_accelerations(), js.get_torques();
  EXPECT_NEAR(concatenated_state.norm(), js.data().norm(), 1e-4);
}

TEST(JointStateTest, JointStateToStdVector) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  std::vector<double> vec_data = js.to_std_vector();
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(js.data()(i), vec_data[i]);
  }
}

TEST(JointStateTest, JointPositionsToStdVector) {
  state_representation::JointPositions jp = state_representation::JointPositions::Random("test_robot", 4);
  std::vector<double> vec_data = jp.to_std_vector();
  EXPECT_EQ(vec_data.size(), jp.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jp.get_positions()(i), vec_data[i]);
  }
}

TEST(JointStateTest, JointVelocitiesToStdVector) {
  state_representation::JointVelocities jv = state_representation::JointVelocities::Random("test_robot", 4);
  std::vector<double> vec_data = jv.to_std_vector();
  EXPECT_EQ(vec_data.size(), jv.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jv.get_velocities()(i), vec_data[i]);
  }
}

TEST(JointStateTest, JointTorquesToStdVector) {
  state_representation::JointTorques jt = state_representation::JointTorques::Random("test_robot", 4);
  std::vector<double> vec_data = jt.to_std_vector();
  EXPECT_EQ(vec_data.size(), jt.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jt.get_torques()(i), vec_data[i]);
  }
}

TEST(JointStateTest, AddTwoState) {
  state_representation::JointState j1 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState j2 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState jsum = j1 + j2;
  EXPECT_NEAR(jsum.data().norm(), (j1.data() + j2.data()).norm(), 1e-4);
}

TEST(JointStateTest, SubstractTwoState) {
  state_representation::JointState j1 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState j2 = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState jdiff = j1 - j2;
  EXPECT_NEAR(jdiff.data().norm(), (j1.data() - j2.data()).norm(), 1e-4);
}

TEST(JointStateTest, MultiplyByScalar) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  state_representation::JointState jscaled = 0.5 * js;
  EXPECT_NEAR(jscaled.data().norm(), (0.5 * js.data()).norm(), 1e-4);
}

TEST(JointStateTest, MultiplyByArray) {
  state_representation::JointState js = state_representation::JointState::Random("test_robot", 4);
  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();
  state_representation::JointState jscaled = gain * js;
  EXPECT_NEAR(jscaled.data().norm(), (gain * js.data()).norm(), 1e-4);
}
