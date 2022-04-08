#include <gtest/gtest.h>
#include "state_representation/space/joint/JointAccelerations.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

static void expect_only_accelerations(JointAccelerations& acc) {
  EXPECT_EQ(static_cast<JointState&>(acc).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(acc).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(acc).get_torques().norm(), 0);
}

TEST(JointAccelerationsTest, Constructors) {
  std::vector<std::string> joint_names{"joint_10", "joint_20"};
  Eigen::Vector2d accelerations = Eigen::Vector2d::Random();
  JointAccelerations ja1("test", accelerations);
  EXPECT_EQ(ja1.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(ja1.get_name(), "test");
  EXPECT_FALSE(ja1.is_empty());
  EXPECT_EQ(ja1.get_size(), accelerations.size());
  for (auto i = 0; i < accelerations.size(); ++i) {
    EXPECT_EQ(ja1.get_names().at(i), "joint" + std::to_string(i));
  }
  EXPECT_EQ(ja1.data(), accelerations);

  JointAccelerations ja2("test", joint_names, accelerations);
  EXPECT_EQ(ja2.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(ja2.get_name(), "test");
  EXPECT_FALSE(ja2.is_empty());
  EXPECT_EQ(ja2.get_size(), joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(ja2.get_names().at(i), joint_names.at(i));
  }
  EXPECT_EQ(ja2.data(), accelerations);
}

TEST(JointAccelerationsTest, StateCopyConstructor) {
  JointState random_state = JointState::Random("test", 3);
  EXPECT_EQ(random_state.get_type(), StateType::JOINT_STATE);
  JointAccelerations copy1(random_state);
  EXPECT_EQ(copy1.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(random_state.get_name(), copy1.get_name());
  EXPECT_EQ(random_state.get_names(), copy1.get_names());
  EXPECT_EQ(random_state.get_size(), copy1.get_size());
  EXPECT_EQ(random_state.get_accelerations(), copy1.data());
  expect_only_accelerations(copy1);

  JointAccelerations copy2 = random_state;
  EXPECT_EQ(copy2.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(random_state.get_name(), copy2.get_name());
  EXPECT_EQ(random_state.get_names(), copy2.get_names());
  EXPECT_EQ(random_state.get_size(), copy2.get_size());
  EXPECT_EQ(random_state.get_accelerations(), copy2.data());
  expect_only_accelerations(copy2);

  JointState empty_state;
  JointAccelerations copy3(empty_state);
  EXPECT_EQ(copy3.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_TRUE(copy3.is_empty());
  JointAccelerations copy4 = empty_state;
  EXPECT_EQ(copy4.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_TRUE(copy4.is_empty());
  JointAccelerations copy5 = empty_state.copy();
  EXPECT_EQ(copy5.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_TRUE(copy5.is_empty());
}

TEST(JointAccelerationsTest, VelocitiesCopyConstructor) {
  JointVelocities random_velocities = JointVelocities::Random("test", 3);
  EXPECT_EQ(random_velocities.get_type(), StateType::JOINT_VELOCITIES);
  JointAccelerations copy1(random_velocities);
  EXPECT_EQ(copy1.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(random_velocities.get_name(), copy1.get_name());
  EXPECT_EQ(random_velocities.get_names(), copy1.get_names());
  EXPECT_EQ(random_velocities.get_size(), copy1.get_size());
  EXPECT_EQ((random_velocities / std::chrono::seconds(1)).data(), copy1.data());
  expect_only_accelerations(copy1);

  JointAccelerations copy2 = random_velocities;
  EXPECT_EQ(copy2.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(random_velocities.get_name(), copy1.get_name());
  EXPECT_EQ(random_velocities.get_names(), copy1.get_names());
  EXPECT_EQ(random_velocities.get_size(), copy1.get_size());
  EXPECT_EQ((random_velocities / std::chrono::seconds(1)).data(), copy1.data());
  expect_only_accelerations(copy2);

  JointVelocities empty_velocities;
  EXPECT_THROW(JointAccelerations copy(empty_velocities), exceptions::EmptyStateException);
  EXPECT_THROW(JointAccelerations copy = empty_velocities, exceptions::EmptyStateException);
}

TEST(JointAccelerationsTest, RandomInitialization) {
  JointAccelerations random1 = JointAccelerations::Random("test", 3);
  EXPECT_EQ(random1.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_NE(random1.get_accelerations().norm(), 0);
  expect_only_accelerations(random1);

  JointAccelerations random2 = JointAccelerations::Random("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_EQ(random2.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_NE(random2.get_accelerations().norm(), 0);
  expect_only_accelerations(random2);
}

TEST(JointAccelerationsTest, Clamping) {
  JointAccelerations ja1("test", 3);
  ja1.set_data(-10 * Eigen::VectorXd::Ones(ja1.get_size()));
  ja1.clamp(15);
  EXPECT_EQ(ja1.data(), -10 * Eigen::VectorXd::Ones(ja1.get_size()));
  ja1.clamp(9);
  EXPECT_EQ(ja1.data(), -9 * Eigen::VectorXd::Ones(ja1.get_size()));
  ja1.clamp(20, 0.5);
  EXPECT_EQ(ja1.data(), Eigen::VectorXd::Zero(ja1.get_size()));

  JointAccelerations ja2("test", 3);
  Eigen::VectorXd accelerations(3), result(3);
  accelerations << -2.0, 1.0, -4.0;
  result << -2.0, 0.0, -3.0;
  ja2.set_data(accelerations);
  ja2.clamp(10);
  EXPECT_EQ(ja2.data(), accelerations);
  ja2.clamp(3 * Eigen::ArrayXd::Ones(ja2.get_size()), 0.5 * Eigen::ArrayXd::Ones(ja2.get_size()));
  EXPECT_EQ(ja2.data(), result);
  EXPECT_EQ(ja2.get_type(), StateType::JOINT_ACCELERATIONS);
}

TEST(JointAccelerationsTest, GetSetData) {
  JointAccelerations ja1 = JointAccelerations::Zero("test", 3);
  JointAccelerations ja2 = JointAccelerations::Random("test", 3);
  Eigen::VectorXd data(ja1.get_size());
  data << ja1.get_accelerations();
  EXPECT_EQ(data, ja1.data());
  for (std::size_t i = 0; i < ja1.get_size(); ++i) {
    EXPECT_EQ(data.array()(i), ja1.array()(i));
  }

  ja1.set_data(ja2.data());
  EXPECT_TRUE(ja2.data().isApprox(ja1.data()));

  auto state_vec = ja2.to_std_vector();
  ja1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_EQ(state_vec.at(i), ja1.data()(i));
  }
  EXPECT_THROW(ja1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
  EXPECT_EQ(ja1.get_type(), StateType::JOINT_ACCELERATIONS);
}

TEST(JointAccelerationsTest, ScalarMultiplication) {
  JointAccelerations ja = JointAccelerations::Random("test", 3);
  JointAccelerations jscaled = 0.5 * ja;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(jscaled.data(), 0.5 * ja.data());

  JointAccelerations empty;
  EXPECT_THROW(0.5 * empty, exceptions::EmptyStateException);
}

TEST(JointAccelerationsTest, MatrixMultiplication) {
  JointAccelerations ja = JointAccelerations::Random("test", 3);
  Eigen::MatrixXd gains = Eigen::VectorXd::Random(ja.get_size()).asDiagonal();

  JointAccelerations jscaled = gains * ja;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(jscaled.data(), gains * ja.data());
  EXPECT_EQ((ja * gains).data(), jscaled.data());
  ja *= gains;
  EXPECT_EQ(ja.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(jscaled.data(), ja.data());
  JointAccelerations jscaled2 = ja * gains;

  gains = Eigen::VectorXd::Random(2 * ja.get_size()).asDiagonal();
  EXPECT_THROW(gains * ja, exceptions::IncompatibleSizeException);
}

TEST(JointAccelerationsTest, ArrayMultiplication) {
  JointAccelerations ja = JointAccelerations::Random("test", 3);
  Eigen::ArrayXd gains = Eigen::ArrayXd::Random(ja.get_size());

  JointAccelerations jscaled = gains * ja;
  EXPECT_EQ(jscaled.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(jscaled.data(), (gains * ja.array()).matrix());
  EXPECT_EQ((ja * gains).data(), jscaled.data());
  ja *= gains;
  EXPECT_EQ(ja.get_type(), StateType::JOINT_ACCELERATIONS);
  EXPECT_EQ(jscaled.data(), ja.data());

  gains = Eigen::ArrayXd::Random(2 * ja.get_size());
  EXPECT_THROW(gains * ja, exceptions::IncompatibleSizeException);
}

TEST(JointAccelerationsTest, ChronoMultiplication) {
  JointAccelerations ja = JointAccelerations::Random("test", 3);
  auto time = std::chrono::seconds(2);
  JointVelocities jv1 = ja * time;
  EXPECT_EQ(jv1.get_type(), StateType::JOINT_VELOCITIES);
  EXPECT_EQ(jv1.get_velocities(), ja.get_accelerations() * time.count());
  JointVelocities jv2 = time * ja;
  EXPECT_EQ(jv2.get_type(), StateType::JOINT_VELOCITIES);
  EXPECT_EQ(jv2.get_velocities(), ja.get_accelerations() * time.count());

  JointAccelerations empty;
  EXPECT_THROW(empty * time, exceptions::EmptyStateException);
}
