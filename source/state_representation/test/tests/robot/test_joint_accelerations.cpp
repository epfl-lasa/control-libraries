#include <gtest/gtest.h>
#include "state_representation/robot/JointAccelerations.hpp"

using namespace state_representation;

TEST(JointAccelerationsTest, RandomAccelerationsInitialization) {
  JointAccelerations random = JointAccelerations::Random("test", 3);
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_velocities().norm(), 0);
  EXPECT_GT(random.get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_torques().norm(), 0);
  // same for the second initializer
  JointAccelerations random2 = JointAccelerations::Random("test", std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_velocities().norm(), 0);
  EXPECT_GT(random2.get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointAccelerationsTest, CopyAccelerations) {
  JointAccelerations accelerations1 = JointAccelerations::Random("test", 3);
  JointAccelerations accelerations2(accelerations1);
  EXPECT_EQ(accelerations1.get_name(), accelerations2.get_name());
  EXPECT_TRUE(accelerations1.data().isApprox(accelerations2.data()));
  EXPECT_EQ(static_cast<JointState&>(accelerations2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations2).get_torques().norm(), 0);
  JointAccelerations accelerations3 = accelerations1;
  EXPECT_EQ(accelerations1.get_name(), accelerations3.get_name());
  EXPECT_TRUE(accelerations1.data().isApprox(accelerations3.data()));
  EXPECT_EQ(static_cast<JointState&>(accelerations3).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations3).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations3).get_torques().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<JointState&>(accelerations1).set_positions(Eigen::Vector3d::Random());
  static_cast<JointState&>(accelerations1).set_velocities(Eigen::Vector3d::Random());
  static_cast<JointState&>(accelerations1).set_torques(Eigen::Vector3d::Random());
  JointAccelerations accelerations4 = accelerations1;
  EXPECT_TRUE(accelerations1.data().isApprox(accelerations4.data()));
  EXPECT_EQ(static_cast<JointState&>(accelerations4).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations4).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations4).get_torques().norm(), 0);
  // copy a state, only the pose variables should be non 0
  JointAccelerations accelerations5 = JointState::Random("test", 3);
  EXPECT_EQ(static_cast<JointState&>(accelerations5).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations5).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(accelerations5).get_torques().norm(), 0);

  JointAccelerations accelerations6;
  EXPECT_TRUE(accelerations6.is_empty());
  JointAccelerations accelerations7 = accelerations6;
  EXPECT_TRUE(accelerations7.is_empty());
}

TEST(JointAccelerationsTest, SetData) {
  JointAccelerations ja1 = JointAccelerations::Zero("test", 4);
  JointAccelerations ja2 = JointAccelerations::Random("test", 4);
  ja1.set_data(ja2.data());
  EXPECT_TRUE(ja2.data().isApprox(ja1.data()));

  auto acc_vec = ja2.to_std_vector();
  ja1.set_data(acc_vec);
  for (std::size_t j = 0; j < acc_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(acc_vec.at(j), ja1.data()(j));
  }
  std::vector<double> accelerations{1, 2, 3, 4, 5};
  EXPECT_THROW(ja1.set_data(accelerations), exceptions::IncompatibleSizeException);
}

TEST(JointAccelerationsTest, JointAccelerationsToStdVector) {
  JointAccelerations ja = JointAccelerations::Random("test_robot", 4);
  std::vector<double> vec_data = ja.to_std_vector();
  EXPECT_EQ(vec_data.size(), ja.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(ja.get_accelerations()(i), vec_data[i]);
  }
}
