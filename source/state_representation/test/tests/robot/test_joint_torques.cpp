#include <gtest/gtest.h>
#include "state_representation/robot/JointTorques.hpp"

using namespace state_representation;

TEST(JointTorquesTest, RandomTorquesInitialization) {
  JointTorques random = JointTorques::Random("test", 3);
  // only torques should be random
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_GT(random.get_torques().norm(), 0);
  // same for the second initializer
  JointTorques random2 = JointTorques::Random("test", std::vector<std::string>{"j0", "j1"});
  // only torques should be random
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_GT(random2.get_torques().norm(), 0);
}

TEST(JointTorquesTest, CopyTorques) {
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

  JointTorques torques6;
  EXPECT_TRUE(torques6.is_empty());
  JointTorques torques7 = torques6;
  EXPECT_TRUE(torques7.is_empty());
}

TEST(JointTorquesTest, SetData) {
  JointTorques jt1 = JointTorques::Zero("test", 4);
  JointTorques jt2 = JointTorques::Random("test", 4);
  jt1.set_data(jt2.data());
  EXPECT_TRUE(jt2.data().isApprox(jt1.data()));

  auto torque_vec = jt2.to_std_vector();
  jt1.set_data(torque_vec);
  for (std::size_t j = 0; j < torque_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(torque_vec.at(j), jt1.data()(j));
  }
}

TEST(JointTorquesTest, JointTorquesToStdVector) {
  JointTorques jt = JointTorques::Random("test_robot", 4);
  std::vector<double> vec_data = jt.to_std_vector();
  EXPECT_EQ(vec_data.size(), jt.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jt.get_torques()(i), vec_data[i]);
  }
}
