#include "dynamical_systems/Circular.hpp"
#include <gtest/gtest.h>
#include <unistd.h>

#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

using namespace std::literals::chrono_literals;

class CircularDSTest : public testing::Test {
protected:
  void SetUp() override {
    current_pose = state_representation::CartesianPose("A", 10 * Eigen::Vector3d::Random());
    center = state_representation::CartesianPose::Identity("B");
  }

  state_representation::CartesianPose current_pose;
  state_representation::CartesianPose center;
  double radius = 10;
  unsigned int nb_steps = 2000;
  std::chrono::milliseconds dt = 10ms;
  double linear_tol = 1e-3;
};

TEST_F(CircularDSTest, TestPositionOnRadius) {
  dynamical_systems::Circular circularDS(center);
  circularDS.set_radius(radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = circularDS.evaluate(current_pose);
    twist.clamp(10, 10, 0.001, 0.001);
    current_pose += dt * twist;
  }

  EXPECT_NEAR((current_pose.get_position() - center.get_position()).norm(), radius, linear_tol);
}

TEST_F(CircularDSTest, SetCenterAndBase) {
  auto BinA = state_representation::CartesianState::Identity("B", "A");
  auto CinA = state_representation::CartesianState::Identity("C", "A");
  auto CinB = state_representation::CartesianState::Identity("C", "B");

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
  EXPECT_THROW(circularDS.set_center(CinB), state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(circularDS.set_center(CinB.inverse()));
}
