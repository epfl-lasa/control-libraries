#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianTwist.hpp"

using namespace state_representation;

static void expect_only_twist(CartesianTwist& twist) {
  EXPECT_EQ(static_cast<CartesianState&>(twist).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist).get_acceleration().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist).get_wrench().norm(), 0);
}

TEST(CartesianTwistTest, RandomTwistInitialization) {
  CartesianTwist random = CartesianTwist::Random("test");
  EXPECT_EQ(random.get_type(), StateType::CARTESIAN_TWIST);
  EXPECT_GT(random.get_twist().norm(), 0);
  expect_only_twist(random);
}

TEST(CartesianTwistTest, CopyTwist) {
  CartesianTwist twist1 = CartesianTwist::Random("test");
  CartesianTwist twist2(twist1);
  EXPECT_EQ(twist1.get_type(), twist2.get_type());
  EXPECT_EQ(twist1.get_name(), twist2.get_name());
  EXPECT_EQ(twist1.get_reference_frame(), twist2.get_reference_frame());
  EXPECT_EQ(twist1.data(), twist2.data());
  expect_only_twist(twist2);

  CartesianTwist twist3 = twist1;
  EXPECT_EQ(twist1.get_type(), twist3.get_type());
  EXPECT_EQ(twist1.get_name(), twist3.get_name());
  EXPECT_EQ(twist1.get_reference_frame(), twist3.get_reference_frame());
  EXPECT_EQ(twist1.data(), twist3.data());
  expect_only_twist(twist3);

  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<CartesianState&>(twist1).set_pose(Eigen::VectorXd::Random(7));
  static_cast<CartesianState&>(twist1).set_acceleration(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(twist1).set_wrench(Eigen::VectorXd::Random(6));
  CartesianTwist twist4 = twist1;
  EXPECT_EQ(twist1.data(), twist4.data());
  expect_only_twist(twist4);

  // copy a state, only the pose variables should be non 0
  CartesianTwist twist5 = CartesianState::Random("test");
  expect_only_twist(twist5);

  CartesianTwist twist6;
  EXPECT_TRUE(twist6.is_empty());
  CartesianTwist twist7 = twist6;
  EXPECT_TRUE(twist7.is_empty());
}

TEST(CartesianTwistTest, SetData) {
  CartesianTwist ct1 = CartesianTwist::Zero("test");
  CartesianTwist ct2 = CartesianTwist::Random("test");
  ct1.set_data(ct2.data());
  EXPECT_TRUE(ct2.data().isApprox(ct1.data()));

  auto twist_vec = ct2.to_std_vector();
  ct1.set_data(twist_vec);
  for (std::size_t j = 0; j < twist_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(twist_vec.at(j), ct1.data()(j));
  }
  std::vector<double> twist{1, 2, 3, 4, 5};
  EXPECT_THROW(ct1.set_data(twist), exceptions::IncompatibleSizeException);
}

TEST(CartesianTwistTest, CartesianTwistToStdVector) {
  CartesianTwist ct = CartesianTwist::Random("test");
  std::vector<double> vec_data = ct.to_std_vector();
  EXPECT_EQ(vec_data.size(), 6);
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(ct.data()(i), vec_data[i]);
  }
}

TEST(CartesianTwistTest, TestVelocityClamping) {
  CartesianTwist vel("test", Eigen::Vector3d(1, -2, 3), Eigen::Vector3d(1, 2, -3));
  vel.clamp(1, 0.5);
  EXPECT_LE(vel.get_linear_velocity().norm(), 1);
  EXPECT_LE(vel.get_angular_velocity().norm(), 0.5);
  vel *= 0.01;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(vel.clamped(1, 0.5, 0.1, 0.1).get_linear_velocity()(i), 0);
    EXPECT_EQ(vel.clamped(1, 0.5, 0.1, 0.1).get_angular_velocity()(i), 0);
  }
}

TEST(CartesianTwistTest, TestTwistNorms) {
  std::vector<double> norms;
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  // independent variables first
  norms = cs.norms(CartesianStateVariable::LINEAR_VELOCITY);
  EXPECT_TRUE(norms.size() == 1);
  EXPECT_NEAR(norms[0], cs.get_linear_velocity().norm(), tolerance);
  norms = cs.norms(CartesianStateVariable::ANGULAR_VELOCITY);
  EXPECT_TRUE(norms.size() == 1);
  EXPECT_NEAR(norms[0], cs.get_angular_velocity().norm(), tolerance);
  // then grouped by two
  norms = cs.norms(CartesianStateVariable::TWIST);
  EXPECT_TRUE(norms.size() == 2);
  EXPECT_NEAR(norms[0], cs.get_linear_velocity().norm(), tolerance);
  EXPECT_NEAR(norms[1], cs.get_angular_velocity().norm(), tolerance);
  // test with CartesianTwist default variable
  CartesianTwist ct = CartesianTwist::Random("ct");
  std::vector<double> twist_norms = ct.norms();
  EXPECT_TRUE(twist_norms.size() == 2);
  EXPECT_NEAR(twist_norms[0], ct.get_linear_velocity().norm(), tolerance);
  EXPECT_NEAR(twist_norms[1], ct.get_angular_velocity().norm(), tolerance);
}

TEST(CartesianTwistTest, TestVelocityToAcceleration) {
  auto twist = CartesianTwist::Random("test");
  std::chrono::seconds dt1(1);
  std::chrono::milliseconds dt2(100);
  auto res1 = twist / dt1;
  EXPECT_EQ(res1.get_type(), StateType::CARTESIAN_ACCELERATION);
  EXPECT_EQ(res1.get_linear_acceleration(), twist.get_linear_velocity());
  EXPECT_EQ(res1.get_angular_acceleration(), twist.get_angular_velocity());
  auto res2 = twist / dt2;
  EXPECT_EQ(res2.get_type(), StateType::CARTESIAN_ACCELERATION);
  EXPECT_TRUE(res2.get_linear_acceleration().isApprox(10 * twist.get_linear_velocity()));
  EXPECT_TRUE(res2.get_angular_acceleration().isApprox(10 * twist.get_angular_velocity()));
}

TEST(CartesianTwistTest, TestImplicitConversion) {
  CartesianAcceleration acc("test");
  acc.set_linear_acceleration(Eigen::Vector3d(0.1, 0.1, 0.1));
  acc.set_linear_acceleration(Eigen::Vector3d(M_PI, 0, 0));
  CartesianTwist twist(acc);
  EXPECT_EQ(twist.get_type(), StateType::CARTESIAN_TWIST);
  EXPECT_EQ(twist.get_linear_velocity(), acc.get_linear_acceleration());
  EXPECT_EQ(twist.get_angular_velocity(), acc.get_angular_acceleration());
}
