#include <gtest/gtest.h>
#include "state_representation/robot/JointVelocities.hpp"

using namespace state_representation;

TEST(JointVelocitiesTest, RandomVelocitiesInitialization) {
  JointVelocities random = JointVelocities::Random("test", 3);
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_torques().norm(), 0);
  // same for the second initializer
  JointVelocities random2 = JointVelocities::Random("test", std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointVelocitiesTest, CopyVelocities) {
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

  JointVelocities velocities6;
  EXPECT_TRUE(velocities6.is_empty());
  JointVelocities velocities7 = velocities6;
  EXPECT_TRUE(velocities7.is_empty());
}

TEST(JointVelocitiesTest, SetData) {
  JointVelocities jv1 = JointVelocities::Zero("test", 4);
  JointVelocities jv2 = JointVelocities::Random("test", 4);
  jv1.set_data(jv2.data());
  EXPECT_TRUE(jv2.data().isApprox(jv1.data()));

  auto vel_vec = jv2.to_std_vector();
  jv1.set_data(vel_vec);
  for (std::size_t j = 0; j < vel_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(vel_vec.at(j), jv1.data()(j));
  }
  std::vector<double> velocities{1, 2, 3, 4, 5};
  EXPECT_THROW(jv1.set_data(velocities), exceptions::IncompatibleSizeException);
}

TEST(JointVelocitiesTest, JointVelocitiesToStdVector) {
  JointVelocities jv = JointVelocities::Random("test_robot", 4);
  std::vector<double> vec_data = jv.to_std_vector();
  EXPECT_EQ(vec_data.size(), jv.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jv.get_velocities()(i), vec_data[i]);
  }
}