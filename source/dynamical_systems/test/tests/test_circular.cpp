#include <gtest/gtest.h>

#include "dynamical_systems/Circular.hpp"
#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"

#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

using namespace state_representation;
using namespace std::literals::chrono_literals;

class CircularDSTest : public testing::Test {
protected:
  void SetUp() override {
    current_pose = CartesianPose("A", 10 * Eigen::Vector3d::Random());
    center = CartesianPose::Identity("B");
  }

  CartesianPose current_pose;
  CartesianPose center;
  double radius = 10;
  unsigned int nb_steps = 2000;
  std::chrono::milliseconds dt = 10ms;
  double linear_tol = 1e-3;
};

TEST_F(CircularDSTest, TestPositionOnRadius) {
  dynamical_systems::Circular circularDS(center);
  circularDS.set_radius(radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = circularDS.evaluate(current_pose);
    twist.clamp(10, 10, 0.001, 0.001);
    current_pose += dt * twist;
  }

  EXPECT_NEAR(current_pose.dist(center, CartesianStateVariable::POSITION), radius, linear_tol);
}

TEST_F(CircularDSTest, EmptyConstructor) {
  // construct empty cartesian state DS
  CartesianPose center = CartesianPose::Identity("CAttractor", "A");
  dynamical_systems::Circular ds;
  // base frame and attractor should be empty
  EXPECT_TRUE(ds.get_center().is_empty());
  EXPECT_TRUE(ds.get_base_frame().is_empty());
  ds.set_center(center);
  EXPECT_FALSE(ds.get_center().is_empty());
  EXPECT_FALSE(ds.get_base_frame().is_empty());
  // when attractor was set without a base frame, expect base frame to be identity with name / reference_frame of attractor
  EXPECT_EQ(ds.get_base_frame().get_name(), center.get_reference_frame());
  EXPECT_EQ(ds.get_base_frame().get_reference_frame(), center.get_reference_frame());
  EXPECT_EQ(ds.get_base_frame().get_transformation_matrix(), Eigen::Matrix4d::Identity());

  ds = dynamical_systems::Circular();
  CartesianState state1 = CartesianState::Identity("B", "A");
  CartesianState state2("D", "C");
  CartesianState state3("C", "A");
  CartesianState state4("C", "B");
  // if no base frame is set, an exception is thrown
  EXPECT_THROW(ds.evaluate(state1), dynamical_systems::exceptions::EmptyBaseFrameException);
  ds.set_base_frame(state1);
  // if cartesian state is incompatible, an exception is thrown
  EXPECT_THROW(ds.evaluate(state2), state_representation::exceptions::IncompatibleReferenceFramesException);
  // if cartesian state needs to be transformed to other frame first and is empty, an exception is thrown
  EXPECT_THROW(ds.evaluate(state3), state_representation::exceptions::EmptyStateException);
  // if no attractor is set, an exception is thrown
  EXPECT_THROW(ds.evaluate(state4), dynamical_systems::exceptions::EmptyAttractorException);

  ds.set_center(center);
  EXPECT_TRUE(ds.is_compatible(state1));
  EXPECT_FALSE(ds.is_compatible(state2));
  EXPECT_TRUE(ds.is_compatible(state3));
  EXPECT_TRUE(ds.is_compatible(state4));
}

TEST_F(CircularDSTest, TestPositionOnRadiusRandomCenter) {
  center.set_position(Eigen::Vector3d::Random());
  center.set_orientation(Eigen::Quaterniond::UnitRandom());
  dynamical_systems::Circular circularDS(center);
  circularDS.set_radius(radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = circularDS.evaluate(current_pose);
    twist.clamp(10, 10, 0.001, 0.001);
    current_pose += dt * twist;
  }

  EXPECT_NEAR(current_pose.dist(center, CartesianStateVariable::POSITION), radius, linear_tol);
}

TEST_F(CircularDSTest, SetCenterAndBase) {
  auto BinA = CartesianState::Identity("B", "A");
  auto CinA = CartesianState::Identity("C", "A");
  auto CinB = CartesianState::Identity("C", "B");

  dynamical_systems::Circular circularDS(BinA);
  EXPECT_STREQ(circularDS.get_center().get_name().c_str(), BinA.get_name().c_str());
  EXPECT_STREQ(circularDS.get_center().get_reference_frame().c_str(), BinA.get_reference_frame().c_str());

  // evaluating a state is only valid if it matches the base reference frame
  EXPECT_NO_THROW(circularDS.evaluate(CinA));
  EXPECT_THROW(circularDS.evaluate(CinB), state_representation::exceptions::IncompatibleReferenceFramesException);

  // setting the center is only valid if it matches the base reference frame
  EXPECT_NO_THROW(circularDS.set_center(CinA));
  EXPECT_THROW(circularDS.set_center(CinB), state_representation::exceptions::IncompatibleReferenceFramesException);

  // setting the base frame should also update the reference frame of the center
  ASSERT_NO_THROW(circularDS.set_base_frame(CinB));
  EXPECT_STREQ(circularDS.get_base_frame().get_name().c_str(), CinB.get_name().c_str());
  EXPECT_STREQ(circularDS.get_base_frame().get_reference_frame().c_str(), CinB.get_reference_frame().c_str());
  EXPECT_STREQ(circularDS.get_center().get_reference_frame().c_str(), circularDS.get_base_frame().get_name().c_str());

  // now the base frame is C, setting a center should only work if the reference frame of the center is also C
  EXPECT_THROW(circularDS.set_center(CinA), state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(circularDS.set_center(CinB));
  EXPECT_NO_THROW(circularDS.set_center(CinB.inverse()));
}
