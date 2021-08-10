#include <fstream>
#include <gtest/gtest.h>
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointTorques.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

using namespace state_representation;

TEST(JointStateTest, ZeroInitialization) {
  JointState zero = JointState::Zero("test", 3);
  // the joint state should not be considered empty (as it is properly initialized)
  EXPECT_FALSE(zero.is_empty());
  // all data should be zero
  EXPECT_EQ(zero.data().norm(), 0);
  // same for the second initializer
  JointState zero2 = JointState::Zero("test", std::vector<std::string>{"j0", "j1"});
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
  JointState random2 = JointState::Random("test", std::vector<std::string>{"j0", "j1"});
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
  JointPositions random2 = JointPositions::Random("test", std::vector<std::string>{"j0", "j1"});
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
  JointVelocities random2 = JointVelocities::Random("test", std::vector<std::string>{"j0", "j1"});
  // only velocities should be random
  EXPECT_EQ(static_cast<JointState&>(random2).get_positions().norm(), 0);
  EXPECT_GT(random2.get_velocities().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<JointState&>(random2).get_torques().norm(), 0);
}

TEST(JointStateTest, RandomAccelerationsInitialization) {
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

TEST(JointStateTest, RandomTorquesInitialization) {
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

TEST(JointStateTest, CopyState) {
  JointState state1 = JointState::Random("test", 7);
  JointState state2(state1);
  EXPECT_EQ(state1.get_name(), state2.get_name());
  EXPECT_TRUE(state1.data().isApprox(state2.data()));
  JointState state3 = state1;
  EXPECT_EQ(state1.get_name(), state3.get_name());
  EXPECT_TRUE(state1.data().isApprox(state3.data()));

  JointState state4;
  EXPECT_TRUE(state4.is_empty());
  JointState state5 = state4;
  EXPECT_TRUE(state5.is_empty());
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

  JointPositions positions6;
  EXPECT_TRUE(positions6.is_empty());
  JointPositions positions7 = positions6;
  EXPECT_TRUE(positions7.is_empty());
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

  JointVelocities velocities6;
  EXPECT_TRUE(velocities6.is_empty());
  JointVelocities velocities7 = velocities6;
  EXPECT_TRUE(velocities7.is_empty());
}

TEST(JointStateTest, CopyAccelerations) {
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

  JointTorques torques6;
  EXPECT_TRUE(torques6.is_empty());
  JointTorques torques7 = torques6;
  EXPECT_TRUE(torques7.is_empty());
}

TEST(JointStateTest, GetData) {
  JointState js = JointState::Random("test_robot", 4);
  Eigen::VectorXd concatenated_state(js.get_size() * 4);
  concatenated_state << js.get_positions(), js.get_velocities(), js.get_accelerations(), js.get_torques();
  EXPECT_TRUE(concatenated_state.isApprox(js.data()));
}

TEST(JointStateTest, SetData) {
  // JointState
  JointState js1 = JointState::Zero("test", 4);
  JointState js2 = JointState::Random("test", 4);
  js1.set_data(js2.data());
  EXPECT_TRUE(js2.data().isApprox(js1.data()));

  auto state_vec = js2.to_std_vector();
  js1.set_data(state_vec);
  for (std::size_t j = 0; j < state_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(state_vec.at(j), js1.data()(j));
  }
  EXPECT_THROW(js1.set_data(Eigen::Vector3d::Zero()), exceptions::IncompatibleSizeException);

  // JointPositions
  JointPositions jp1 = JointPositions::Zero("test", 4);
  JointPositions jp2 = JointPositions::Random("test", 4);
  jp1.set_data(jp2.data());
  EXPECT_TRUE(jp2.data().isApprox(jp1.data()));

  auto pos_vec = jp2.to_std_vector();
  jp1.set_data(pos_vec);
  for (std::size_t j = 0; j < pos_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(pos_vec.at(j), jp1.data()(j));
  }

  // JointVelocities
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

  // JointAccelerations
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

  // JointTorques
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

TEST(JointStateTest, JointAccelerationsToStdVector) {
  JointAccelerations ja = JointAccelerations::Random("test_robot", 4);
  std::vector<double> vec_data = ja.to_std_vector();
  EXPECT_EQ(vec_data.size(), ja.get_size());
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(ja.get_accelerations()(i), vec_data[i]);
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
