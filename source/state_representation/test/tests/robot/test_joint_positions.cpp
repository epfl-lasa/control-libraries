#include <gtest/gtest.h>
#include "state_representation/robot/JointPositions.hpp"

using namespace state_representation;

TEST(JointPositionsTest, RandomPositionsInitialization) {
  JointPositions random = JointPositions::Random("test", 3);
  // only position should be random
  EXPECT_GT(random.get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_torques().norm(), 0);
  // same for the second initializer
  JointPositions random2 = JointPositions::Random("test", std::vector<std::string>{"j0", "j1"});
  // only position should be random
  EXPECT_GT(random2.get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointPositionsTest, CopyPosisitions) {
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

  JointPositions positions6;
  EXPECT_TRUE(positions6.is_empty());
  JointPositions positions7 = positions6;
  EXPECT_TRUE(positions7.is_empty());
}

TEST(JointPositionsTest, SetData) {
  JointPositions jp1 = JointPositions::Zero("test", 4);
  JointPositions jp2 = JointPositions::Random("test", 4);
  jp1.set_data(jp2.data());
  EXPECT_TRUE(jp2.data().isApprox(jp1.data()));

  auto pos_vec = jp2.to_std_vector();
  jp1.set_data(pos_vec);
  for (std::size_t j = 0; j < pos_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(pos_vec.at(j), jp1.data()(j));
  }
}

TEST(JointPositionsTest, JointPositionsToStdVector) {
  JointPositions jp = JointPositions::Random("test_robot", 4);
  std::vector<double> vec_data = jp.to_std_vector();
  EXPECT_EQ(vec_data.size(), jp.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(jp.get_positions()(i), vec_data[i]);
  }
}
