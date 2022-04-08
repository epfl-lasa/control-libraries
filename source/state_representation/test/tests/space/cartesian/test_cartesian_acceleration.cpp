#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianAcceleration.hpp"

using namespace state_representation;

static void expect_only_acceleration(CartesianAcceleration& acceleration) {
  EXPECT_EQ(static_cast<CartesianState&>(acceleration).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(acceleration).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(acceleration).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(acceleration).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(acceleration).get_wrench().norm(), 0);
}

TEST(CartesianAccelerationTest, RandomAccelerationInitialization) {
  CartesianAcceleration random = CartesianAcceleration::Random("test");
  EXPECT_EQ(random.get_type(), StateType::CARTESIAN_ACCELERATION);
  EXPECT_GT(random.get_acceleration().norm(), 0);
  expect_only_acceleration(random);
}

TEST(CartesianAccelerationTest, CopyAcceleration) {
  CartesianAcceleration acc1 = CartesianAcceleration::Random("test");
  CartesianAcceleration acc2(acc1);
  EXPECT_EQ(acc1.get_type(), acc2.get_type());
  EXPECT_EQ(acc1.get_name(), acc2.get_name());
  EXPECT_EQ(acc1.get_reference_frame(), acc2.get_reference_frame());
  EXPECT_EQ(acc1.data(), acc2.data());
  expect_only_acceleration(acc2);

  CartesianAcceleration acc3 = acc1;
  EXPECT_EQ(acc1.get_type(), acc3.get_type());
  EXPECT_EQ(acc1.get_name(), acc3.get_name());
  EXPECT_EQ(acc1.get_reference_frame(), acc3.get_reference_frame());
  EXPECT_EQ(acc1.data(), acc3.data());
  expect_only_acceleration(acc3);

  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<CartesianState&>(acc1).set_pose(Eigen::VectorXd::Random(7));
  static_cast<CartesianState&>(acc1).set_acceleration(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(acc1).set_wrench(Eigen::VectorXd::Random(6));
  CartesianAcceleration acc4 = acc1;
  EXPECT_EQ(acc1.data(), acc4.data());
  expect_only_acceleration(acc4);

  // copy a state, only the pose variables should be non 0
  CartesianAcceleration acc5 = CartesianState::Random("test");
  expect_only_acceleration(acc5);

  CartesianAcceleration acc6;
  EXPECT_TRUE(acc6.is_empty());
  CartesianAcceleration acc7 = acc6;
  EXPECT_TRUE(acc7.is_empty());
}

TEST(CartesianAccelerationTest, SetData) {
  CartesianAcceleration ca1 = CartesianAcceleration::Zero("test");
  CartesianAcceleration ca2 = CartesianAcceleration::Random("test");
  ca1.set_data(ca2.data());
  EXPECT_TRUE(ca2.data().isApprox(ca2.data()));

  auto acc_vec = ca2.to_std_vector();
  ca1.set_data(acc_vec);
  for (std::size_t j = 0; j < acc_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(acc_vec.at(j), ca1.data()(j));
  }
  std::vector<double> acc{1, 2, 3, 4, 5};
  EXPECT_THROW(ca1.set_data(acc), exceptions::IncompatibleSizeException);
}

TEST(CartesianAccelerationTest, CartesianAccelerationToStdVector) {
  CartesianAcceleration ca = CartesianAcceleration::Random("test");
  std::vector<double> vec_data = ca.to_std_vector();
  EXPECT_EQ(vec_data.size(), 6);
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(ca.data()(i), vec_data[i]);
  }
}

TEST(CartesianAccelerationTest, TestAccelerationClamping) {
  CartesianAcceleration acc("test", Eigen::Vector3d(1, -2, 3), Eigen::Vector3d(1, 2, -3));
  acc.clamp(1, 0.5);
  EXPECT_LE(acc.get_linear_acceleration().norm(), 1);
  EXPECT_LE(acc.get_angular_acceleration().norm(), 0.5);
  acc *= 0.01;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(acc.clamped(1, 0.5, 0.1, 0.1).get_linear_acceleration()(i), 0);
    EXPECT_EQ(acc.clamped(1, 0.5, 0.1, 0.1).get_angular_acceleration()(i), 0);
  }
}

TEST(CartesianAccelerationTest, TestAccelerationNorms) {
  std::vector<double> norms;
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  // independent variables first
  norms = cs.norms(CartesianStateVariable::LINEAR_ACCELERATION);
  EXPECT_TRUE(norms.size() == 1);
  EXPECT_NEAR(norms[0], cs.get_linear_acceleration().norm(), tolerance);
  norms = cs.norms(CartesianStateVariable::ANGULAR_ACCELERATION);
  EXPECT_TRUE(norms.size() == 1);
  EXPECT_NEAR(norms[0], cs.get_angular_acceleration().norm(), tolerance);
  // then grouped by two
  norms = cs.norms(CartesianStateVariable::ACCELERATION);
  EXPECT_TRUE(norms.size() == 2);
  EXPECT_NEAR(norms[0], cs.get_linear_acceleration().norm(), tolerance);
  EXPECT_NEAR(norms[1], cs.get_angular_acceleration().norm(), tolerance);
  // test with CartesianAcceleration default variable
  CartesianAcceleration ct = CartesianTwist::Random("ct");
  std::vector<double> twist_norms = ct.norms();
  EXPECT_TRUE(twist_norms.size() == 2);
  EXPECT_NEAR(twist_norms[0], ct.get_linear_acceleration().norm(), tolerance);
  EXPECT_NEAR(twist_norms[1], ct.get_angular_acceleration().norm(), tolerance);
}
