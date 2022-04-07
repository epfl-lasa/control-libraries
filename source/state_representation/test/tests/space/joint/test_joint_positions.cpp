#include <gtest/gtest.h>
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

static void expect_only_positions(JointPositions& pos) {
  EXPECT_EQ(static_cast<JointState&>(pos).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(pos).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(pos).get_torques().norm(), 0);
}

TEST(JointPositionsTest, Constructors) {
  std::vector<std::string> joint_names{"joint_10", "joint_20"};
  Eigen::Vector2d positions = Eigen::Vector2d::Random();
  JointPositions jp1("test", positions);
  EXPECT_EQ(jp1.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jp1.get_name(), "test");
  EXPECT_FALSE(jp1.is_empty());
  EXPECT_EQ(jp1.get_size(), positions.size());
  for (auto i = 0; i < positions.size(); ++i) {
    EXPECT_EQ(jp1.get_names().at(i), "joint" + std::to_string(i));
  }
  EXPECT_EQ(jp1.data(), positions);

  JointPositions jp2("test", joint_names, positions);
  EXPECT_EQ(jp2.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jp2.get_name(), "test");
  EXPECT_FALSE(jp2.is_empty());
  EXPECT_EQ(jp2.get_size(), joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(jp2.get_names().at(i), joint_names.at(i));
  }
  EXPECT_EQ(jp2.data(), positions);
}

TEST(JointPositionsTest, StateCopyConstructor) {
  JointState random_state = JointState::Random("test", 3);
  EXPECT_EQ(random_state.get_type(), StateType::JOINT_STATE);
  JointPositions copy1(random_state);
  EXPECT_EQ(copy1.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(random_state.get_name(), copy1.get_name());
  EXPECT_EQ(random_state.get_names(), copy1.get_names());
  EXPECT_EQ(random_state.get_size(), copy1.get_size());
  EXPECT_EQ(random_state.get_positions(), copy1.data());
  expect_only_positions(copy1);

  JointPositions copy2 = random_state;
  EXPECT_EQ(copy2.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(random_state.get_name(), copy2.get_name());
  EXPECT_EQ(random_state.get_names(), copy2.get_names());
  EXPECT_EQ(random_state.get_size(), copy2.get_size());
  EXPECT_EQ(random_state.get_positions(), copy2.data());
  expect_only_positions(copy2);

  JointState empty_state;
  JointPositions copy3(empty_state);
  EXPECT_EQ(copy3.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_TRUE(copy3.is_empty());
  JointPositions copy4 = empty_state;
  EXPECT_EQ(copy4.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_TRUE(copy4.is_empty());
  JointPositions copy5 = empty_state.copy();
  EXPECT_EQ(copy5.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_TRUE(copy5.is_empty());
}

TEST(JointPositionsTest, VelocitiesCopyConstructor) {
  JointVelocities random_velocities = JointVelocities::Random("test", 3);
  EXPECT_EQ(random_velocities.get_type(), StateType::JOINT_VELOCITIES);
  JointPositions copy1(random_velocities);
  EXPECT_EQ(copy1.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(random_velocities.get_name(), copy1.get_name());
  EXPECT_EQ(random_velocities.get_names(), copy1.get_names());
  EXPECT_EQ(random_velocities.get_size(), copy1.get_size());
  EXPECT_EQ((std::chrono::seconds(1) * random_velocities).data(), copy1.data());
  expect_only_positions(copy1);

  JointPositions copy2 = random_velocities;
  EXPECT_EQ(copy2.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(random_velocities.get_name(), copy1.get_name());
  EXPECT_EQ(random_velocities.get_names(), copy1.get_names());
  EXPECT_EQ(random_velocities.get_size(), copy1.get_size());
  EXPECT_EQ((std::chrono::seconds(1) * random_velocities).data(), copy1.data());
  expect_only_positions(copy2);

  JointVelocities empty_velocities;
  EXPECT_THROW(JointPositions copy(empty_velocities), exceptions::EmptyStateException);
  EXPECT_THROW(JointPositions copy = empty_velocities, exceptions::EmptyStateException);
}

TEST(JointPositionsTest, RandomInitialization) {
  JointPositions random1 = JointPositions::Random("test", 3);
  EXPECT_EQ(random1.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_NE(random1.get_positions().norm(), 0);
  expect_only_positions(random1);

  JointPositions random2 = JointPositions::Random("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_EQ(random2.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_NE(random2.get_positions().norm(), 0);
  expect_only_positions(random2);
}

TEST(JointPositionsTest, Clamping) {
  JointPositions jp1("test", 3);
  jp1.set_data(-10 * Eigen::VectorXd::Ones(jp1.get_size()));
  jp1.clamp(15);
  EXPECT_EQ(jp1.data(), -10 * Eigen::VectorXd::Ones(jp1.get_size()));
  jp1.clamp(9);
  EXPECT_EQ(jp1.data(), -9 * Eigen::VectorXd::Ones(jp1.get_size()));
  jp1.clamp(20, 0.5);
  EXPECT_EQ(jp1.data(), Eigen::VectorXd::Zero(jp1.get_size()));

  JointPositions jp2("test", 3);
  Eigen::VectorXd positions(3), result(3);
  positions << -2.0, 1.0, -4.0;
  result << -2.0, 0.0, -3.0;
  jp2.set_data(positions);
  jp2.clamp(10);
  EXPECT_EQ(jp2.data(), positions);
  jp2.clamp(3 * Eigen::ArrayXd::Ones(jp2.get_size()), 0.5 * Eigen::ArrayXd::Ones(jp2.get_size()));
  EXPECT_EQ(jp2.data(), result);
}

TEST(JointPositionsTest, GetSetData) {
  JointPositions jp1 = JointPositions::Zero("test", 3);
  JointPositions jp2 = JointPositions::Random("test", 3);
  Eigen::VectorXd data(jp1.get_size());
  data << jp1.get_positions();
  EXPECT_EQ(data, jp1.data());
  for (std::size_t i = 0; i < jp1.get_size(); ++i) {
    EXPECT_EQ(data.array()(i), jp1.array()(i));
  }

  jp1.set_data(jp2.data());
  EXPECT_TRUE(jp2.data().isApprox(jp1.data()));

  auto state_vec = jp2.to_std_vector();
  jp1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_EQ(state_vec.at(i), jp1.data()(i));
  }
  EXPECT_THROW(jp1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(JointPositionsTest, ScalarMultiplication) {
  JointPositions jp = JointPositions::Random("test", 3);
  JointPositions jscaled = 0.5 * jp;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jscaled.data(), 0.5 * jp.data());

  JointPositions empty;
  EXPECT_THROW(0.5 * empty, exceptions::EmptyStateException);
}

TEST(JointPositionsTest, MatrixMultiplication) {
  JointPositions jp = JointPositions::Random("test", 3);
  Eigen::MatrixXd gains = Eigen::VectorXd::Random(jp.get_size()).asDiagonal();

  JointPositions jscaled = gains * jp;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jscaled.data(), gains * jp.data());
  EXPECT_EQ((jp * gains).data(), jscaled.data());
  jp *= gains;
  EXPECT_EQ(jp.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jscaled.data(), jp.data());
  JointPositions jscaled2 = jp * gains;

  gains = Eigen::VectorXd::Random(2 * jp.get_size()).asDiagonal();
  EXPECT_THROW(gains * jp, exceptions::IncompatibleSizeException);
}

TEST(JointPositionsTest, ArrayMultiplication) {
  JointPositions jp = JointPositions::Random("test", 3);
  Eigen::ArrayXd gains = Eigen::ArrayXd::Random(jp.get_size());

  JointPositions jscaled = gains * jp;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jscaled.data(), (gains * jp.array()).matrix());
  EXPECT_EQ((jp * gains).data(), jscaled.data());
  jp *= gains;
  EXPECT_EQ(jp.get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(jscaled.data(), jp.data());

  gains = Eigen::ArrayXd::Random(2 * jp.get_size());
  EXPECT_THROW(gains * jp, exceptions::IncompatibleSizeException);
}

TEST(JointStateTest, ChronoDivision) {
  JointPositions jp = JointPositions::Random("test", 3);
  EXPECT_EQ(jp.get_type(), StateType::JOINT_POSITIONS);
  auto time = std::chrono::seconds(1);
  JointVelocities jv = jp / time;
  EXPECT_EQ(jv.get_type(), StateType::JOINT_VELOCITIES);
  EXPECT_EQ(jv.get_velocities(), jp.get_positions() / time.count());

  JointPositions empty;
  EXPECT_THROW(empty / time, exceptions::EmptyStateException);
}
