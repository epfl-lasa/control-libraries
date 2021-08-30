#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianTwist.hpp"

using namespace state_representation;

TEST(CartesianTwistTest, RandomTwistInitialization) {
  CartesianTwist random = CartesianTwist::Random("test");
  // only position should be random
  EXPECT_EQ(static_cast<CartesianState&>(random).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_orientation().w(), 1);
  EXPECT_GT(random.get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_wrench().norm(), 0);
}

TEST(CartesianTwistTest, CopyTwist) {
  CartesianTwist twist1 = CartesianTwist::Random("test");
  CartesianTwist twist2(twist1);
  EXPECT_EQ(twist1.get_name(), twist2.get_name());
  EXPECT_EQ(twist1.get_reference_frame(), twist2.get_reference_frame());
  EXPECT_TRUE(twist1.data().isApprox(twist2.data()));
  EXPECT_EQ(static_cast<CartesianState&>(twist2).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist2).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist2).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist2).get_wrench().norm(), 0);
  CartesianTwist twist3 = twist1;
  EXPECT_EQ(twist1.get_name(), twist3.get_name());
  EXPECT_EQ(twist1.get_reference_frame(), twist3.get_reference_frame());
  EXPECT_TRUE(twist1.data().isApprox(twist3.data()));
  EXPECT_EQ(static_cast<CartesianState&>(twist3).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist3).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist3).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist3).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist3).get_wrench().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<CartesianState&>(twist1).set_pose(Eigen::VectorXd::Random(7));
  static_cast<CartesianState&>(twist1).set_accelerations(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(twist1).set_wrench(Eigen::VectorXd::Random(6));
  CartesianTwist twist4 = twist1;
  EXPECT_TRUE(twist1.data().isApprox(twist4.data()));
  EXPECT_EQ(static_cast<CartesianState&>(twist4).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist4).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist4).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist4).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist4).get_wrench().norm(), 0);
  // copy a state, only the pose variables should be non 0
  CartesianTwist twist5 = CartesianState::Random("test");
  EXPECT_EQ(static_cast<CartesianState&>(twist5).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist5).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist5).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(twist5).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(twist5).get_wrench().norm(), 0);

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
