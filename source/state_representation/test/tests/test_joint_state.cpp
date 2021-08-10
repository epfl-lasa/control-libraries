#include <fstream>
#include <gtest/gtest.h>
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation;

TEST(JointState, Constructors) {
  JointState empty;
  EXPECT_EQ(empty.get_name(), "none");
  EXPECT_EQ(empty.get_type(), StateType::JOINTSTATE);
  EXPECT_TRUE(empty.is_empty());
  EXPECT_EQ(empty.get_size(), 0);
  EXPECT_EQ(empty.data().norm(), 0);

  JointState js1("test", 3);
  EXPECT_EQ(js1.get_name(), "test");
  EXPECT_TRUE(js1.is_empty());
  EXPECT_EQ(js1.get_size(), 3);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(js1.get_names().at(i), "joint" + std::to_string(i));
  }
  EXPECT_EQ(js1.data().norm(), 0);

  std::vector<std::string> joint_names{"joint_10", "joint_20"};
  JointState js2("test", joint_names);
  EXPECT_EQ(js2.get_name(), "test");
  EXPECT_TRUE(js2.is_empty());
  EXPECT_EQ(js2.get_size(), joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(js2.get_names().at(i), joint_names.at(i));
  }
  EXPECT_EQ(js2.data().norm(), 0);
}

TEST(JointState, ZeroInitialization) {
  JointState zero = JointState::Zero("test", 3);
  EXPECT_FALSE(zero.is_empty());
  EXPECT_EQ(zero.data().norm(), 0);

  JointState zero2 = JointState::Zero("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_FALSE(zero2.is_empty());
  EXPECT_EQ(zero2.data().norm(), 0);
}

TEST(JointState, RandomStateInitialization) {
  JointState random = JointState::Random("test", 3);
  EXPECT_GT(random.get_positions().norm(), 0);
  EXPECT_GT(random.get_velocities().norm(), 0);
  EXPECT_GT(random.get_accelerations().norm(), 0);
  EXPECT_GT(random.get_torques().norm(), 0);

  JointState random2 = JointState::Random("test", std::vector<std::string>{"j0", "j1"});
  EXPECT_GT(random2.get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_GT(random2.get_accelerations().norm(), 0);
  EXPECT_GT(random2.get_torques().norm(), 0);
}

TEST(JointState, CopyConstructor) {
  JointState random = JointState::Random("test", 3);
  JointState copy1(random);
  EXPECT_EQ(random.get_name(), copy1.get_name());
  EXPECT_EQ(random.get_names(), copy1.get_names());
  EXPECT_EQ(random.get_size(), copy1.get_size());
  EXPECT_EQ(random.data(), copy1.data());

  JointState copy2 = random;
  EXPECT_EQ(random.get_name(), copy2.get_name());
  EXPECT_EQ(random.get_names(), copy2.get_names());
  EXPECT_EQ(random.get_size(), copy2.get_size());
  EXPECT_EQ(random.data(), copy2.data());

  JointState copy3 = random.copy();
  EXPECT_EQ(random.get_name(), copy3.get_name());
  EXPECT_EQ(random.get_names(), copy3.get_names());
  EXPECT_EQ(random.get_size(), copy3.get_size());
  EXPECT_EQ(random.data(), copy3.data());

  JointState empty;
  JointState copy4(empty);
  EXPECT_TRUE(copy4.is_empty());
  JointState copy5 = empty;
  EXPECT_TRUE(copy5.is_empty());
  JointState copy6 = empty.copy();
  EXPECT_TRUE(copy6.is_empty());
}

TEST(JointState, GetSetFields) {
  JointState js("test", 3);

  // name
  js.set_name("robot");
  EXPECT_EQ(js.get_name(), "robot");
  std::vector<std::string> joint_names{"j1", "j2", "j3"};
  js.set_names(joint_names);
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    EXPECT_EQ(js.get_names().at(i), joint_names.at(i));
  }
  joint_names.emplace_back("j4");
  EXPECT_THROW(js.set_names(4), exceptions::IncompatibleSizeException);
  EXPECT_THROW(js.set_names(joint_names), exceptions::IncompatibleSizeException);

  // fields
  std::vector<double> positions{1, 2, 3};
  js.set_positions(positions);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    EXPECT_EQ(js.get_positions()(i), positions.at(i));
  }
  std::vector<double> velocities{4, 5, 6};
  js.set_velocities(velocities);
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    EXPECT_EQ(js.get_velocities()(i), velocities.at(i));
  }
  std::vector<double> accelerations{7, 8, 9};
  js.set_accelerations(accelerations);
  for (std::size_t i = 0; i < accelerations.size(); ++i) {
    EXPECT_EQ(js.get_accelerations()(i), accelerations.at(i));
  }
  std::vector<double> torques{10, 11, 12};
  js.set_torques(torques);
  for (std::size_t i = 0; i < torques.size(); ++i) {
    EXPECT_EQ(js.get_torques()(i), torques.at(i));
  }
  EXPECT_THROW(js.set_positions(Eigen::VectorXd::Zero(4)), exceptions::IncompatibleSizeException);
  EXPECT_THROW(js.set_velocities(Eigen::VectorXd::Zero(5)), exceptions::IncompatibleSizeException);
  EXPECT_THROW(js.set_accelerations(Eigen::VectorXd::Zero(6)), exceptions::IncompatibleSizeException);
  EXPECT_THROW(js.set_torques(Eigen::VectorXd::Zero(7)), exceptions::IncompatibleSizeException);

  js.set_zero();
  EXPECT_EQ(js.data().norm(), 0);
  EXPECT_EQ(js.is_empty(), false);
  js.set_empty();
  EXPECT_EQ(js.is_empty(), true);
}

TEST(JointState, Compatibility) {
  JointState js1("test", 3);
  JointState js2("test", std::vector<std::string>{"j1", "j2", "j3"});
  JointState js3("test", 4);
  JointState js4("robot", 3);

  EXPECT_FALSE(js1.is_compatible(js2));
  EXPECT_FALSE(js1.is_compatible(js3));
  EXPECT_FALSE(js1.is_compatible(js4));
}

TEST(JointState, GetData) {
  JointState js = JointState::Random("test", 3);
  Eigen::VectorXd concatenated_state(js.get_size() * 4);
  concatenated_state << js.get_positions(), js.get_velocities(), js.get_accelerations(), js.get_torques();
  EXPECT_TRUE(concatenated_state.isApprox(js.data()));
}

TEST(JointState, SetData) {
  JointState js1 = JointState::Zero("test", 3);
  JointState js2 = JointState::Random("test", 3);
  js1.set_data(js2.data());
  EXPECT_TRUE(js2.data().isApprox(js1.data()));

  auto state_vec = js2.to_std_vector();
  js1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_FLOAT_EQ(state_vec.at(i), js1.data()(i));
  }
  EXPECT_THROW(js1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(JointState, JointStateToStdVector) {
  JointState js = JointState::Random("test", 3);
  std::vector<double> vec_data = js.to_std_vector();
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(js.data()(i), vec_data[i]);
  }
}

TEST(JointState, ClampVariable) {
  JointState js("test", 3);
  js.set_data(-10 * Eigen::VectorXd::Ones(js.get_size() * 4));
  js.clamp_state_variable(9, JointStateVariable::ALL);
  for (std::size_t i = 0; i < js.get_size() * 4; ++i) {
    EXPECT_EQ(js.data()(i), -9);
  }
  js.set_data(10 * Eigen::VectorXd::Ones(js.get_size() * 4));
  js.clamp_state_variable(9, JointStateVariable::ALL);
  for (std::size_t i = 0; i < js.get_size() * 4; ++i) {
    EXPECT_EQ(js.data()(i), 9);
  }

  js.clamp_state_variable(8, JointStateVariable::POSITIONS);
  for (std::size_t i = 0; i < js.get_size(); ++i) {
    EXPECT_EQ(js.get_positions()(i), 8);
  }

  Eigen::Array3d max_absolute_value_array(4, 5, 6);
  js.clamp_state_variable(max_absolute_value_array, JointStateVariable::VELOCITIES, Eigen::Array3d::Zero());
  for (std::size_t i = 0; i < js.get_size(); ++i) {
    EXPECT_EQ(js.get_velocities()(i), max_absolute_value_array(i));
  }

  js.clamp_state_variable(100 * Eigen::Array3d::Ones(), JointStateVariable::ACCELERATIONS, 10 * Eigen::Array3d::Ones());
  for (std::size_t i = 0; i < js.get_size(); ++i) {
    EXPECT_EQ(js.get_accelerations()(i), 0);
  }

  js.clamp_state_variable(10 * Eigen::Array3d::Ones(), JointStateVariable::TORQUES, 3 * Eigen::Array3d::Ones());
  for (std::size_t i = 0; i < js.get_size(); ++i) {
    EXPECT_EQ(js.get_torques()(i), 9);
  }

  EXPECT_THROW(js.clamp_state_variable(Eigen::Array2d::Ones(), JointStateVariable::ALL, Eigen::Array2d::Zero()),
               IncompatibleSizeException);
}

TEST(JointState, Distance) {
  JointState js;
  JointState js1 = JointState::Random("test", 3);
  JointState js2 = JointState::Random("test", 2);
  EXPECT_THROW(js.dist(js1), EmptyStateException);
  EXPECT_THROW(js1.dist(js), EmptyStateException);
  EXPECT_THROW(js1.dist(js2), IncompatibleStatesException);

  Eigen::VectorXd data1 = Eigen::VectorXd::Random(js1.get_size() * 4);
  js1.set_data(data1);
  JointState js3 = JointState("test", 3);
  Eigen::VectorXd data2 = Eigen::VectorXd::Random(js1.get_size() * 4);
  js3.set_data(data2);

  EXPECT_FLOAT_EQ(js1.dist(js3, JointStateVariable::POSITIONS), (data1.head(3) - data2.head(3)).norm());
  EXPECT_FLOAT_EQ(js1.dist(js3, JointStateVariable::VELOCITIES), (data1.segment(3, 3) - data2.segment(3, 3)).norm());
  EXPECT_FLOAT_EQ(js1.dist(js3, JointStateVariable::ACCELERATIONS), (data1.segment(6, 3) - data2.segment(6, 3)).norm());
  EXPECT_FLOAT_EQ(js1.dist(js3, JointStateVariable::TORQUES), (data1.tail(3) - data2.tail(3)).norm());
  EXPECT_FLOAT_EQ(js1.dist(js3), js3.dist(js1));
}

//TEST(JointStateTest, AddTwoState) {
//  JointState j1 = JointState::Random("test_robot", 4);
//  JointState j2 = JointState::Random("test_robot", 4);
//  JointState jsum = j1 + j2;
//  EXPECT_NEAR(jsum.data().norm(), (j1.data() + j2.data()).norm(), 1e-4);
//}

//TEST(JointStateTest, SubstractTwoState) {
//  JointState j1 = JointState::Random("test_robot", 4);
//  JointState j2 = JointState::Random("test_robot", 4);
//  JointState jdiff = j1 - j2;
//  EXPECT_NEAR(jdiff.data().norm(), (j1.data() - j2.data()).norm(), 1e-4);
//}
//
//TEST(JointStateTest, MultiplyByScalar) {
//  JointState js = JointState::Random("test_robot", 4);
//  JointState jscaled = 0.5 * js;
//  EXPECT_NEAR(jscaled.data().norm(), (0.5 * js.data()).norm(), 1e-4);
//}
//
//TEST(JointStateTest, MultiplyByArray) {
//  JointState js = JointState::Random("test_robot", 4);
//  Eigen::MatrixXd gain = Eigen::VectorXd::Random(16).asDiagonal();
//  JointState jscaled = gain * js;
//  EXPECT_NEAR(jscaled.data().norm(), (gain * js.data()).norm(), 1e-4);
//}

TEST(JointStateTest, StateClamping) {
  JointState js = JointState("test_robot", 4);
  js.set_data(10 * Eigen::VectorXd::Ones(4 * 4));
  js.clamp_state_variable(9, JointStateVariable::ALL, 0);
  EXPECT_EQ(js.data(), 9 * Eigen::VectorXd::Ones(4 * 4));

  js.clamp_state_variable(10, JointStateVariable::POSITIONS, 0.5);
  EXPECT_EQ(js.get_positions(), 9 * Eigen::VectorXd::Ones(4));
  js.set_positions(2 * Eigen::VectorXd::Ones(4));
  js.clamp_state_variable(9, JointStateVariable::POSITIONS, 0.5);
  EXPECT_EQ(js.get_positions(), Eigen::VectorXd::Zero(4));

  Eigen::VectorXd accelerations(4), result(4);
  accelerations << -2.0, 1.0, -4.0, 4.0;
  result << -2.0, 0.0, -3.0, 3.0;
  js.set_accelerations(accelerations);
  js.clamp_state_variable(10, JointStateVariable::ACCELERATIONS);
  EXPECT_EQ(js.get_accelerations(), accelerations);
  js.clamp_state_variable(
      3 * Eigen::ArrayXd::Ones(4), JointStateVariable::ACCELERATIONS, 0.5 * Eigen::ArrayXd::Ones(4));
  EXPECT_EQ(js.get_accelerations(), result);
}
