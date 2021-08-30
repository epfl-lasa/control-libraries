#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianWrench.hpp"

using namespace state_representation;

TEST(CartesianWrenchTest, RandomWrenchInitialization) {
  CartesianWrench random = CartesianWrench::Random("test");
  // only position should be random
  EXPECT_EQ(static_cast<CartesianState&>(random).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_accelerations().norm(), 0);
  EXPECT_GT(random.get_wrench().norm(), 0);
}

TEST(CartesianWrenchTest, CopyWrench) {
  CartesianWrench wrench1 = CartesianWrench::Random("test");
  CartesianWrench wrench2(wrench1);
  EXPECT_EQ(wrench1.get_name(), wrench2.get_name());
  EXPECT_EQ(wrench1.get_reference_frame(), wrench2.get_reference_frame());
  EXPECT_TRUE(wrench1.data().isApprox(wrench2.data()));
  EXPECT_EQ(static_cast<CartesianState&>(wrench2).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench2).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench2).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench2).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench2).get_accelerations().norm(), 0);
  CartesianWrench wrench3 = wrench1;
  EXPECT_EQ(wrench1.get_name(), wrench3.get_name());
  EXPECT_EQ(wrench1.get_reference_frame(), wrench3.get_reference_frame());
  EXPECT_TRUE(wrench1.data().isApprox(wrench3.data()));
  EXPECT_EQ(static_cast<CartesianState&>(wrench3).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench3).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench3).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench3).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench3).get_accelerations().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<CartesianState&>(wrench1).set_pose(Eigen::VectorXd::Random(7));
  static_cast<CartesianState&>(wrench1).set_twist(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(wrench1).set_accelerations(Eigen::VectorXd::Random(6));
  CartesianWrench wrench4 = wrench1;
  EXPECT_TRUE(wrench1.data().isApprox(wrench4.data()));
  EXPECT_EQ(static_cast<CartesianState&>(wrench4).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench4).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench4).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench4).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench4).get_accelerations().norm(), 0);
  // copy a state, only the pose variables should be non 0
  CartesianWrench wrench5 = CartesianState::Random("test");
  EXPECT_EQ(static_cast<CartesianState&>(wrench5).get_position().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench5).get_orientation().norm(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench5).get_orientation().w(), 1);
  EXPECT_EQ(static_cast<CartesianState&>(wrench5).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(wrench5).get_accelerations().norm(), 0);

  CartesianWrench wrench6;
  EXPECT_TRUE(wrench6.is_empty());
  CartesianWrench wrench7 = wrench6;
  EXPECT_TRUE(wrench7.is_empty());
}

TEST(CartesianWrenchTest, SetData) {
  CartesianWrench cw1 = CartesianWrench::Zero("test");
  CartesianWrench cw2 = CartesianWrench::Random("test");
  cw1.set_data(cw2.data());
  EXPECT_TRUE(cw2.data().isApprox(cw1.data()));

  auto wrench_vec = cw2.to_std_vector();
  cw1.set_data(wrench_vec);
  for (std::size_t j = 0; j < wrench_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(wrench_vec.at(j), cw1.data()(j));
  }
}

TEST(CartesianWrenchTest, CartesianWrenchToStdVector) {
  CartesianWrench cw = CartesianWrench::Random("test");
  std::vector<double> vec_data = cw.to_std_vector();
  EXPECT_EQ(vec_data.size(), 6);
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(cw.data()(i), vec_data[i]);
  }
}

TEST(CartesianWrenchTest, TestWrenchNorms) {
  std::vector<double> norms;
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  // independent variables first
  norms = cs.norms(CartesianStateVariable::FORCE);
  EXPECT_EQ(norms.size(), 1);
  EXPECT_NEAR(norms[0], cs.get_force().norm(), tolerance);
  norms = cs.norms(CartesianStateVariable::TORQUE);
  EXPECT_EQ(norms.size(), 1);
  EXPECT_NEAR(norms[0], cs.get_torque().norm(), tolerance);
  // then grouped by two
  norms = cs.norms(CartesianStateVariable::WRENCH);
  EXPECT_EQ(norms.size(), 2);
  EXPECT_NEAR(norms[0], cs.get_force().norm(), tolerance);
  EXPECT_NEAR(norms[1], cs.get_torque().norm(), tolerance);
  // test with CartesianTwist default variable
  CartesianWrench cw = CartesianWrench::Random("cw");
  std::vector<double> wrench_norms = cw.norms();
  EXPECT_EQ(wrench_norms.size(), 2);
  EXPECT_NEAR(wrench_norms[0], cw.get_force().norm(), tolerance);
  EXPECT_NEAR(wrench_norms[1], cw.get_torque().norm(), tolerance);
}
