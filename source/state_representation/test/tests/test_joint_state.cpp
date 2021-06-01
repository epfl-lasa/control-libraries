#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

using namespace state_representation;

TEST(JointStateTest, ZeroInitialization) {
  JointState zero = JointState::Zero("test", 3);
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero.is_empty());
  // all data should be zero
  EXPECT_EQ(zero.data().norm(), 0);
  // same for the second initializer
  JointState zero2 = JointState::Zero("test",
                                      std::vector<std::string>{"j0", "j1"});
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero2.is_empty());
  // all data should be zero
  EXPECT_EQ(zero2.data().norm(), 0);
  // should not be able to change the names attribute with a wrong size
  EXPECT_THROW(zero.set_names(5), exceptions::IncompatibleSizeException);
  EXPECT_THROW(zero2.set_names(std::vector<std::string>{"j0", "j1", "j2"}), exceptions::IncompatibleSizeException);
}

TEST(JointStateTest, RandomStateInitialization) {
  JointState random = JointState::Random("test", 3);
  // all data should be random (non 0)
  EXPECT_GT(random.get_positions().norm(), 0);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_GT(random.get_accelerations().norm(), 0);
  EXPECT_GT(random.get_torques().norm(), 0);
  // same for the second initializer
  JointState random2 =
      JointState::Random("test",
                         std::vector<std::string>{"j0", "j1"});
  // all data should be random (non 0)
  EXPECT_GT(random2.get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_GT(random2.get_accelerations().norm(), 0);
  EXPECT_GT(random2.get_torques().norm(), 0);
}

TEST(JointStateTest, RandomPositionsInitialization) {
  JointPositions random = JointPositions::Random("test", 3);
  // only position should be random
  EXPECT_GT(random.get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_torques().norm(), 0);
  // same for the second initializer
  JointPositions random2 =
      JointPositions::Random("test",
                             std::vector<std::string>{"j0", "j1"});
  // only position should be random
  EXPECT_GT(random2.get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointStateTest, RandomVelocitiesInitialization) {
  JointVelocities random = JointVelocities::Random("test", 3);
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_torques().norm(), 0);
  // same for the second initializer
  JointVelocities random2 =
      JointVelocities::Random("test",
                              std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointStateTest, RandomTorquesInitialization) {
  JointTorques random = JointTorques::Random("test", 3);
  // only torques should be random
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_GT(random.get_torques().norm(), 0);
  // same for the second initializer
  JointTorques random2 =
      JointTorques::Random("test",
                           std::vector<std::string>{"j0", "j1"});
  // only torques should be random
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_GT(random2.get_torques().norm(), 0);
}

TEST(JointStateTest, CopyState) {
  JointState state1 = JointState::Random("test", 7);
  JointState state2(state1);
  EXPECT_EQ(state1.get_name(), state2.get_name());
  EXPECT_TRUE(state1.data().isApprox(state2.data()));
  JointState state3 = state1;
  EXPECT_EQ(state1.get_name(), state3.get_name());
  EXPECT_TRUE(state1.data().isApprox(state3.data()));
}

TEST(JointStateTest, CopyPosisitions) {
  JointPositions positions1 = JointPositions::Random("test", 3);
  JointPositions positions2(positions1);
  EXPECT_EQ(positions1.get_name(), positions2.get_name());
  EXPECT_TRUE(positions1.data().isApprox(positions2.data()));
  EXPECT_EQ(static_cast<JointState&>(positions2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions2).get_torques().norm(), 0);
  JointPositions positions3 = positions1;
  EXPECT_EQ(positions1.get_name(), positions3.get_name());
  EXPECT_TRUE(positions1.data().isApprox(positions3.data()));
  EXPECT_EQ(static_cast<JointState&>(positions3).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions3).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions3).get_torques().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<JointState&>(positions1).set_velocities(Eigen::Vector3d::Random());
  static_cast<JointState&>(positions1).set_accelerations(Eigen::Vector3d::Random());
  static_cast<JointState&>(positions1).set_torques(Eigen::Vector3d::Random());
  JointPositions positions4 = positions1;
  EXPECT_TRUE(positions1.data().isApprox(positions4.data()));
  EXPECT_EQ(static_cast<JointState&>(positions4).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions4).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions4).get_torques().norm(), 0);
  // copy a state, only the pose variables should be non 0
  JointPositions positions5 = JointState::Random("test", 3);
  EXPECT_EQ(static_cast<JointState&>(positions5).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions5).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(positions5).get_torques().norm(), 0);
}

TEST(JointStateTest, CopyVelocities) {
  JointVelocities velocities1 = JointVelocities::Random("test", 3);
  JointVelocities velocities2(velocities1);
  EXPECT_EQ(velocities1.get_name(), velocities2.get_name());
  EXPECT_TRUE(velocities1.data().isApprox(velocities2.data()));
  EXPECT_EQ(static_cast<JointState&>(velocities2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities2).get_torques().norm(), 0);
  JointVelocities velocities3 = velocities1;
  EXPECT_EQ(velocities1.get_name(), velocities3.get_name());
  EXPECT_TRUE(velocities1.data().isApprox(velocities3.data()));
  EXPECT_EQ(static_cast<JointState&>(velocities3).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities3).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities3).get_torques().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<JointState&>(velocities1).set_positions(Eigen::Vector3d::Random());
  static_cast<JointState&>(velocities1).set_accelerations(Eigen::Vector3d::Random());
  static_cast<JointState&>(velocities1).set_torques(Eigen::Vector3d::Random());
  JointVelocities velocities4 = velocities1;
  EXPECT_TRUE(velocities1.data().isApprox(velocities4.data()));
  EXPECT_EQ(static_cast<JointState&>(velocities4).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities4).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities4).get_torques().norm(), 0);
  // copy a state, only the pose variables should be non 0
  JointVelocities velocities5 = JointState::Random("test", 3);
  EXPECT_EQ(static_cast<JointState&>(velocities5).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities5).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(velocities5).get_torques().norm(), 0);
}

TEST(JointStateTest, CopyTorques) {
  JointTorques torques1 = JointTorques::Random("test", 3);
  JointTorques torques2(torques1);
  EXPECT_EQ(torques1.get_name(), torques2.get_name());
  EXPECT_TRUE(torques1.data().isApprox(torques2.data()));
  EXPECT_EQ(static_cast<JointState&>(torques2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques2).get_accelerations().norm(), 0);
  JointTorques torques3 = torques1;
  EXPECT_EQ(torques1.get_name(), torques3.get_name());
  EXPECT_TRUE(torques1.data().isApprox(torques3.data()));
  EXPECT_EQ(static_cast<JointState&>(torques3).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques3).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques3).get_accelerations().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<JointState&>(torques1).set_positions(Eigen::Vector3d::Random());
  static_cast<JointState&>(torques1).set_velocities(Eigen::Vector3d::Random());
  static_cast<JointState&>(torques1).set_accelerations(Eigen::Vector3d::Random());
  JointTorques torques4 = torques1;
  EXPECT_TRUE(torques1.data().isApprox(torques4.data()));
  EXPECT_EQ(static_cast<JointState&>(torques4).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques4).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques4).get_accelerations().norm(), 0);
  // copy a state, only the pose variables should be non 0
  JointTorques torques5 = JointState::Random("test", 3);
  EXPECT_EQ(static_cast<JointState&>(torques5).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques5).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(torques5).get_accelerations().norm(), 0);
}

TEST(JointStateTest, GetData) {
  JointState js = JointState::Random("test_robot", 4);
  Eigen::VectorXd concatenated_state(js.get_size() * 4);
  concatenated_state << js.get_positions(), js.get_velocities(), js.get_accelerations(), js.get_torques();
  EXPECT_NEAR(concatenated_state.norm(), js.data().norm(), 1e-4);
}

TEST(JointStateTest, JointStateToStdVector) {
  JointState js = JointState::Random("test_robot", 4);
  std::vector<double> vec_data = js.to_std_vector();
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(js.data()(i), vec_data[i]);
  }
}

TEST(JointStateTest, JointPositionsToStdVector) {
  JointPositions jp = JointPositions::Random("test_robot", 4);
  std::vector<double> vec_data = jp.to_std_vector();
  EXPECT_EQ(vec_data.size(), jp.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jp.get_positions()(i), vec_data[i]);
  }
}

TEST(JointStateTest, JointVelocitiesToStdVector) {
  JointVelocities jv = JointVelocities::Random("test_robot", 4);
  std::vector<double> vec_data = jv.to_std_vector();
  EXPECT_EQ(vec_data.size(), jv.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jv.get_velocities()(i), vec_data[i]);
  }
}

TEST(JointStateTest, JointTorquesToStdVector) {
  JointTorques jt = JointTorques::Random("test_robot", 4);
  std::vector<double> vec_data = jt.to_std_vector();
  EXPECT_EQ(vec_data.size(), jt.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jt.get_torques()(i), vec_data[i]);
  }
}

TEST(JointStateTest, AddTwoState) {
  JointState j1 = JointState::Random("test_robot", 4);
  JointState j2 = JointState::Random("test_robot", 4);
  JointState jsum = j1 + j2;
  EXPECT_NEAR(jsum.data().norm(), (j1.data() + j2.data()).norm(), 1e-4);
}

TEST(JointStateTest, SubstractTwoState) {
  JointState j1 = JointState::Random("test_robot", 4);
  JointState j2 = JointState::Random("test_robot", 4);
  JointState jdiff = j1 - j2;
  EXPECT_NEAR(jdiff.data().norm(), (j1.data() - j2.data()).norm(), 1e-4);
}

TEST(JointStateTest, MultiplyByScalar) {
  JointState js = JointState::Random("test_robot", 4);
  JointState jscaled = 0.5 * js;
  EXPECT_NEAR(jscaled.data().norm(), (0.5 * js.data()).norm(), 1e-4);
}

TEST(JointStateTest, MultiplyByArray) {
  JointState js = JointState::Random("test_robot", 4);
  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();
  JointState jscaled = gain * js;
  EXPECT_NEAR(jscaled.data().norm(), (gain * js.data()).norm(), 1e-4);
}
