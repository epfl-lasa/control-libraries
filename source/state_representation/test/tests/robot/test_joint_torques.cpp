#include <gtest/gtest.h>
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

TEST(JointTorquesTest, Constructors) {
  std::vector<std::string> joint_names{"joint_10", "joint_20"};
  Eigen::Vector2d torques = Eigen::Vector2d::Random();
  JointTorques jt1("test", torques);
  EXPECT_EQ(jt1.get_name(), "test");
  EXPECT_FALSE(jt1.is_empty());
  EXPECT_EQ(jt1.get_size(), torques.size());
  for (std::size_t i = 0; i < torques.size(); ++i) {
    EXPECT_EQ(jt1.get_names().at(i), "joint" + std::to_string(i));
  }
  EXPECT_EQ(jt1.data(), torques);

  JointTorques jt2("test", joint_names, torques);
  EXPECT_EQ(jt2.get_name(), "test");
  EXPECT_FALSE(jt2.is_empty());
  EXPECT_EQ(jt2.get_size(), joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(jt2.get_names().at(i), joint_names.at(i));
  }
  EXPECT_EQ(jt2.data(), torques);
}

TEST(JointTorquesTest, StateCopyConstructor) {
  JointState random_state = JointState::Random("test", 3);
  JointTorques copy1(random_state);
  EXPECT_EQ(random_state.get_name(), copy1.get_name());
  EXPECT_EQ(random_state.get_names(), copy1.get_names());
  EXPECT_EQ(random_state.get_size(), copy1.get_size());
  EXPECT_EQ(random_state.get_torques(), copy1.data());
  EXPECT_EQ(static_cast<JointState&>(copy1).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy1).get_accelerations().norm(), 0);

  JointTorques copy2 = random_state;
  EXPECT_EQ(random_state.get_name(), copy2.get_name());
  EXPECT_EQ(random_state.get_names(), copy2.get_names());
  EXPECT_EQ(random_state.get_size(), copy2.get_size());
  EXPECT_EQ(random_state.get_torques(), copy2.data());
  EXPECT_EQ(static_cast<JointState&>(copy2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(copy2).get_accelerations().norm(), 0);

  JointState empty_state;
  JointTorques copy3(empty_state);
  EXPECT_TRUE(copy3.is_empty());
  JointTorques copy4 = empty_state;
  EXPECT_TRUE(copy4.is_empty());
  JointTorques copy5 = empty_state.copy();
  EXPECT_TRUE(copy5.is_empty());
}

TEST(JointTorquesTest, RandomInitialization) {
  JointTorques random = JointTorques::Random("test", 3);
  EXPECT_GT(random.get_torques().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random).get_accelerations().norm(), 0);

  JointTorques random2 = JointTorques::Random("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_GT(random2.get_torques().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
}

TEST(JointTorquesTest, Clamping) {
  JointTorques jt1("test", 3);
  jt1.set_data(-10 * Eigen::VectorXd::Ones(jt1.get_size()));
  jt1.clamp(15);
  EXPECT_EQ(jt1.data(), -10 * Eigen::VectorXd::Ones(jt1.get_size()));
  jt1.clamp(9);
  EXPECT_EQ(jt1.data(), -9 * Eigen::VectorXd::Ones(jt1.get_size()));
  jt1.clamp(20, 0.5);
  EXPECT_EQ(jt1.data(), Eigen::VectorXd::Zero(jt1.get_size()));

  JointTorques jt2("test", 3);
  Eigen::VectorXd torques(3), result(3);
  torques << -2.0, 1.0, -4.0;
  result << -2.0, 0.0, -3.0;
  jt2.set_data(torques);
  jt2.clamp(10);
  EXPECT_EQ(jt2.data(), torques);
  jt2.clamp(3 * Eigen::ArrayXd::Ones(jt2.get_size()), 0.5 * Eigen::ArrayXd::Ones(jt2.get_size()));
  EXPECT_EQ(jt2.data(), result);
}

TEST(JointTorquesTest, GetSetData) {
  JointTorques jt1 = JointTorques::Zero("test", 3);
  JointTorques jt2 = JointTorques::Random("test", 3);
  Eigen::VectorXd data(jt1.get_size());
  data << jt1.get_torques();
  EXPECT_EQ(data, jt1.data());
  for (std::size_t i = 0; i < jt1.get_size(); ++i) {
    EXPECT_EQ(data.array()(i), jt1.array()(i));
  }

  jt1.set_data(jt2.data());
  EXPECT_TRUE(jt2.data().isApprox(jt1.data()));

  auto state_vec = jt2.to_std_vector();
  jt1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_EQ(state_vec.at(i), jt1.data()(i));
  }
  EXPECT_THROW(jt1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(JointTorquesTest, ScalarMultiplication) {
  JointTorques jt = JointTorques::Random("test", 3);
  JointTorques jscaled = 0.5 * jt;
  EXPECT_EQ(jscaled.data(), 0.5 * jt.data());

  JointTorques empty;
  EXPECT_THROW(0.5 * empty, exceptions::EmptyStateException);
}

TEST(JointTorquesTest, MatrixMultiplication) {
  JointTorques jt = JointTorques::Random("test", 3);
  Eigen::MatrixXd gains = Eigen::VectorXd::Random(jt.get_size()).asDiagonal();

  JointTorques jscaled = gains * jt;
  EXPECT_EQ(jscaled.data(), gains * jt.data());
  EXPECT_EQ((jt * gains).data(), jscaled.data());
  jt *= gains;
  EXPECT_EQ(jscaled.data(), jt.data());
  JointTorques jscaled2 = jt * gains;

  gains = Eigen::VectorXd::Random(2 * jt.get_size()).asDiagonal();
  EXPECT_THROW(gains * jt, exceptions::IncompatibleSizeException);
}

TEST(JointTorquesTest, ArrayMultiplication) {
  JointTorques jt = JointTorques::Random("test", 3);
  Eigen::ArrayXd gains = Eigen::ArrayXd::Random(jt.get_size());

  JointTorques jscaled = gains * jt;
  EXPECT_EQ(jscaled.data(), (gains * jt.array()).matrix());
  EXPECT_EQ((jt * gains).data(), jscaled.data());
  jt *= gains;
  EXPECT_EQ(jscaled.data(), jt.data());

  gains = Eigen::ArrayXd::Random(2 * jt.get_size());
  EXPECT_THROW(gains * jt, exceptions::IncompatibleSizeException);
}
