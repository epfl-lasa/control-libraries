#include <gtest/gtest.h>
#include "state_representation/robot/JointVelocities.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

TEST(JointVelocitiesTest, Constructors) {
  std::vector<std::string> joint_names{"joint_10", "joint_20"};
  Eigen::Vector2d velocities = Eigen::Vector2d::Random();
  JointVelocities jv1("test", velocities);
  EXPECT_EQ(jv1.get_name(), "test");
  EXPECT_FALSE(jv1.is_empty());
  EXPECT_EQ(jv1.get_size(), velocities.size());
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    EXPECT_EQ(jv1.get_names().at(i), "joint" + std::to_string(i));
  }
  EXPECT_EQ(jv1.data(), velocities);

  JointVelocities jv2("test", joint_names, velocities);
  EXPECT_EQ(jv2.get_name(), "test");
  EXPECT_FALSE(jv2.is_empty());
  EXPECT_EQ(jv2.get_size(), joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(jv2.get_names().at(i), joint_names.at(i));
  }
  EXPECT_EQ(jv2.data(), velocities);
}

TEST(JointVelocitiesTest, StateCopyConstructor) {
  JointState random_state = JointState::Random("test", 3);
  JointVelocities copy1(random_state);
  EXPECT_EQ(random_state.get_name(), copy1.get_name());
  EXPECT_EQ(random_state.get_names(), copy1.get_names());
  EXPECT_EQ(random_state.get_size(), copy1.get_size());
  EXPECT_EQ(random_state.get_velocities(), copy1.data());
  EXPECT_EQ(static_cast<JointState&>(copy1).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_torques().norm(), 0);

  JointVelocities copy2 = random_state;
  EXPECT_EQ(random_state.get_name(), copy2.get_name());
  EXPECT_EQ(random_state.get_names(), copy2.get_names());
  EXPECT_EQ(random_state.get_size(), copy2.get_size());
  EXPECT_EQ(random_state.get_velocities(), copy2.data());
  EXPECT_EQ(static_cast<JointState&>(copy2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_torques().norm(), 0);

  JointState empty_state;
  JointVelocities copy3(empty_state);
  EXPECT_TRUE(copy3.is_empty());
  JointVelocities copy4 = empty_state;
  EXPECT_TRUE(copy4.is_empty());
  JointVelocities copy5 = empty_state.copy();
  EXPECT_TRUE(copy5.is_empty());
}

TEST(JointVelocitiesTest, AccelerationsCopyConstructor) {
  JointAccelerations random_accelerations = JointAccelerations::Random("test", 3);
  JointVelocities copy1(random_accelerations);
  EXPECT_EQ(random_accelerations.get_name(), copy1.get_name());
  EXPECT_EQ(random_accelerations.get_names(), copy1.get_names());
  EXPECT_EQ(random_accelerations.get_size(), copy1.get_size());
  EXPECT_EQ((std::chrono::seconds(1) * random_accelerations).data(), copy1.data());
  EXPECT_EQ(static_cast<JointState&>(copy1).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_torques().norm(), 0);

  JointVelocities copy2 = random_accelerations;
  EXPECT_EQ(random_accelerations.get_name(), copy1.get_name());
  EXPECT_EQ(random_accelerations.get_names(), copy1.get_names());
  EXPECT_EQ(random_accelerations.get_size(), copy1.get_size());
  EXPECT_EQ((std::chrono::seconds(1) * random_accelerations).data(), copy1.data());
  EXPECT_EQ(static_cast<JointState&>(copy2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_torques().norm(), 0);

  JointAccelerations empty_accelerations;
  EXPECT_THROW(JointVelocities copy(empty_accelerations), exceptions::EmptyStateException);
  EXPECT_THROW(JointVelocities copy = empty_accelerations, exceptions::EmptyStateException);
}

TEST(JointVelocitiesTest, PositionsCopyConstructor) {
  JointPositions random_positions = JointPositions::Random("test", 3);
  JointVelocities copy1(random_positions);
  EXPECT_EQ(random_positions.get_name(), copy1.get_name());
  EXPECT_EQ(random_positions.get_names(), copy1.get_names());
  EXPECT_EQ(random_positions.get_size(), copy1.get_size());
  EXPECT_EQ((random_positions / std::chrono::seconds(1)).data(), copy1.data());
  EXPECT_EQ(static_cast<JointState&>(copy1).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_torques().norm(), 0);

  JointVelocities copy2 = random_positions;
  EXPECT_EQ(random_positions.get_name(), copy1.get_name());
  EXPECT_EQ(random_positions.get_names(), copy1.get_names());
  EXPECT_EQ(random_positions.get_size(), copy1.get_size());
  EXPECT_EQ((random_positions / std::chrono::seconds(1)).data(), copy1.data());
  EXPECT_EQ(static_cast<JointState&>(copy2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_torques().norm(), 0);

  JointPositions empty_positions;
  EXPECT_THROW(JointVelocities copy(empty_positions), exceptions::EmptyStateException);
  EXPECT_THROW(JointVelocities copy = empty_positions, exceptions::EmptyStateException);
}

TEST(JointVelocitiesTest, RandomInitialization) {
  JointVelocities random = JointVelocities::Random("test", 3);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_torques().norm(), 0);

  JointVelocities random2 = JointVelocities::Random("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointVelocitiesTest, Clamping) {
  JointVelocities jv1("test", 3);
  jv1.set_data(-10 * Eigen::VectorXd::Ones(jv1.get_size()));
  jv1.clamp(15);
  EXPECT_EQ(jv1.data(), -10 * Eigen::VectorXd::Ones(jv1.get_size()));
  jv1.clamp(9);
  EXPECT_EQ(jv1.data(), -9 * Eigen::VectorXd::Ones(jv1.get_size()));
  jv1.clamp(20, 0.5);
  EXPECT_EQ(jv1.data(), Eigen::VectorXd::Zero(jv1.get_size()));

  JointVelocities jv2("test", 3);
  Eigen::VectorXd velocities(3), result(3);
  velocities << -2.0, 1.0, -4.0;
  result << -2.0, 0.0, -3.0;
  jv2.set_data(velocities);
  jv2.clamp(10);
  EXPECT_EQ(jv2.data(), velocities);
  jv2.clamp(3 * Eigen::ArrayXd::Ones(jv2.get_size()), 0.5 * Eigen::ArrayXd::Ones(jv2.get_size()));
  EXPECT_EQ(jv2.data(), result);
}

TEST(JointVelocitiesTest, GetSetData) {
  JointVelocities jv1 = JointVelocities::Zero("test", 3);
  JointVelocities jv2 = JointVelocities::Random("test", 3);
  Eigen::VectorXd data(jv1.get_size());
  data << jv1.get_velocities();
  EXPECT_EQ(data, jv1.data());
  for (std::size_t i = 0; i < jv1.get_size(); ++i) {
    EXPECT_EQ(data.array()(i), jv1.array()(i));
  }

  jv1.set_data(jv2.data());
  EXPECT_TRUE(jv2.data().isApprox(jv1.data()));

  auto state_vec = jv2.to_std_vector();
  jv1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_EQ(state_vec.at(i), jv1.data()(i));
  }
  EXPECT_THROW(jv1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(JointVelocitiesTest, ScalarMultiplication) {
  JointVelocities jv = JointVelocities::Random("test", 3);
  JointVelocities jscaled = 0.5 * jv;
  EXPECT_EQ(jscaled.data(), 0.5 * jv.data());

  JointVelocities empty;
  EXPECT_THROW(0.5 * empty, exceptions::EmptyStateException);
}

TEST(JointVelocitiesTest, MatrixMultiplication) {
  JointVelocities jv = JointVelocities::Random("test", 3);
  Eigen::MatrixXd gains = Eigen::VectorXd::Random(jv.get_size()).asDiagonal();

  JointVelocities jscaled = gains * jv;
  EXPECT_EQ(jscaled.data(), gains * jv.data());
  EXPECT_EQ((jv * gains).data(), jscaled.data());
  jv *= gains;
  EXPECT_EQ(jscaled.data(), jv.data());
  JointVelocities jscaled2 = jv * gains;

  gains = Eigen::VectorXd::Random(2 * jv.get_size()).asDiagonal();
  EXPECT_THROW(gains * jv, exceptions::IncompatibleSizeException);
}

TEST(JointVelocitiesTest, ArrayMultiplication) {
  JointVelocities jv = JointVelocities::Random("test", 3);
  Eigen::ArrayXd gains = Eigen::ArrayXd::Random(jv.get_size());

  JointVelocities jscaled = gains * jv;
  EXPECT_EQ(jscaled.data(), (gains * jv.array()).matrix());
  EXPECT_EQ((jv * gains).data(), jscaled.data());
  jv *= gains;
  EXPECT_EQ(jscaled.data(), jv.data());

  gains = Eigen::ArrayXd::Random(2 * jv.get_size());
  EXPECT_THROW(gains * jv, exceptions::IncompatibleSizeException);
}

TEST(JointVelocitiesTest, ChronoMultiplication) {
  JointVelocities jv = JointVelocities::Random("test", 3);
  auto time = std::chrono::seconds(2);
  JointPositions jp1 = jv * time;
  EXPECT_EQ(jp1.get_positions(), jv.get_velocities() * time.count());
  JointPositions jp2 = time * jv;
  EXPECT_EQ(jp2.get_positions(), jv.get_velocities() * time.count());

  JointVelocities empty;
  EXPECT_THROW(empty * time, exceptions::EmptyStateException);
}

TEST(JointVelocitiesTest, ChronoDivision) {
  JointVelocities jv = JointVelocities::Random("test", 3);
  auto time = std::chrono::seconds(2);
  JointAccelerations ja = jv / time;
  EXPECT_EQ(ja.get_accelerations(), jv.get_velocities() / time.count());

  JointVelocities empty;
  EXPECT_THROW(empty / time, exceptions::EmptyStateException);
}
