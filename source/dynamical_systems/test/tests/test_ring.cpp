#include <gtest/gtest.h>

#include "dynamical_systems/Ring.hpp"

#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

using namespace std::literals::chrono_literals;

class RingDSTest : public ::testing::Test {
protected:
  RingDSTest() {
    center = state_representation::CartesianPose::Identity("A");
    current_pose = state_representation::CartesianPose("B", radius * Eigen::Vector3d::Random());
  }

  state_representation::CartesianPose current_pose;
  state_representation::CartesianPose center;
  std::vector<double> vlim = {10, 10, 0.001, 0.001};
  double radius = 10;
  double width = 1;
  double speed = 1;
  double field_strength = 0.5;
  unsigned int nb_steps = 1000;
  std::chrono::milliseconds dt = 10ms;
  double tol = 1e-3;
};

TEST_F(RingDSTest, PointsOnRadius) {
  dynamical_systems::Ring ring(center, radius, width, speed);
  state_representation::CartesianTwist twist;

  // zero output at center
  current_pose = center;
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.data().norm(), 0, tol);

  current_pose.set_position(radius, 0, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), speed, tol);

  current_pose.set_position(0, radius, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), -speed, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(-radius, 0, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), -speed, tol);

  current_pose.set_position(0, -radius, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), speed, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(radius, 0, 1);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().z(), -1, tol);
}


TEST_F(RingDSTest, PointsNearRadius) {
  dynamical_systems::Ring ring(center, radius, width, speed, field_strength);
  state_representation::CartesianTwist twist;

  current_pose.set_position(radius + width, 0, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), -speed * field_strength, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(radius - width, 0, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), speed * field_strength, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(0, radius + width, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), -speed * field_strength, tol);

  current_pose.set_position(0, radius - width, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), speed * field_strength, tol);

  current_pose.set_position(radius + (width * 2), 0, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), -speed * field_strength, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);
}

TEST_F(RingDSTest, BehaviourNearBoundaryAtHighSpeeds) {
  radius = 1;
  width = 0.1;
  speed = 10;
  field_strength = 2;
  dynamical_systems::Ring ring(center, radius, width, speed, field_strength);
  state_representation::CartesianTwist twist;

  current_pose.set_position(0, radius + 1.001 * width, 0);
  twist = ring.evaluate(current_pose);
  double angular_speed = twist.get_angular_velocity().norm();
  current_pose.set_position(0, radius + 1.000 * width, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), angular_speed, 0.1);
  current_pose.set_position(0, radius + 0.999 * width, 0);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), angular_speed, 0.1);
}

TEST_F(RingDSTest, ConvergenceOnRadius) {
  dynamical_systems::Ring ring(center, radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = ring.evaluate(current_pose);
    twist.clamp(vlim[0], vlim[1], vlim[2], vlim[3]);
    current_pose += dt * twist;
  }

  EXPECT_NEAR((current_pose.get_position() - center.get_position()).norm(), radius, tol);
}

TEST_F(RingDSTest, ConvergenceOnRadiusRandomCenter) {
  center.set_position(Eigen::Vector3d::Random());
  center.set_orientation(Eigen::Quaterniond::UnitRandom());
  dynamical_systems::Ring ring(center, radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = ring.evaluate(current_pose);
    twist.clamp(vlim[0], vlim[1], vlim[2], vlim[3]);
    current_pose += dt * twist;
  }

  EXPECT_NEAR((current_pose.get_position() - center.get_position()).norm(), radius, tol);
}

TEST_F(RingDSTest, ZeroNormalGain) {
  dynamical_systems::Ring ring(center);
  ring.set_normal_gain(0);

  double startingHeight = current_pose.get_position().z();
  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = ring.evaluate(current_pose);
    twist.clamp(vlim[0], vlim[1], vlim[2], vlim[3]);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.get_position().z(), startingHeight, tol);
}

TEST_F(RingDSTest, OrientationAroundCircle) {
  dynamical_systems::Ring ring(center, radius, width, 0);
  state_representation::CartesianTwist twist;

  // at the position {radius, 0, 0}, the orientation attractor is by default null,
  // so there should be zero angular velocity if the current pose orientation is also null
  current_pose.set_position(radius, 0, 0);
  current_pose.set_orientation(Eigen::Quaterniond::Identity());
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // at the position {0, radius, 0}, the default rotation around Z is pi/2
  current_pose.set_position(0, radius, 0);
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 0, 1));
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // at the position {0, radius, 0}, the default rotation around Z is pi
  current_pose.set_position(-radius, 0, 0);
  current_pose.set_orientation(Eigen::Quaterniond(0, 0, 0, 1));
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // at the position {0, radius, 0}, the default rotation around Z is 3 * pi/2
  current_pose.set_position(0, -radius, 0);
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 0, -1));
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);
}


TEST_F(RingDSTest, OrientationRestitutionAtZeroAngle) {
  dynamical_systems::Ring ring(center, radius, width, 0);
  state_representation::CartesianTwist twist;

  // at the position {radius, 0, 0}, the orientation attractor is by default null (angle around circle is 0)
  current_pose.set_position(radius, 0, 0);

  // rotate the current pose around local X
  current_pose.set_orientation(Eigen::Quaterniond(1, 1, 0, 0));
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), -M_PI_2 * ring.get_angular_gain(), tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);

  // rotate the current pose around local Y
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 1, 0));
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), -M_PI_2 * ring.get_angular_gain(), tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);

  // rotate the current pose around local Z
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 0, 1));
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), -M_PI_2 * ring.get_angular_gain(), tol);
}

TEST_F(RingDSTest, OrientationRotationOffset) {
  dynamical_systems::Ring ring(center, radius, width, 0);
  state_representation::CartesianTwist twist;

  current_pose.set_position(radius, 0, 0);

  Eigen::Quaterniond rotation = Eigen::Quaterniond(1, 0, 1, 0).normalized(); // Eigen::Quaterniond::UnitRandom();

  // if the rotation offset is the same as the current orientation, the angular velocity at
  // position {radius, 0, 0} is always zero
  current_pose.set_orientation(rotation);
  ring.set_rotation_offset(rotation);
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // now if the current pose has some orientation relative to the rotation offset,
  // it will yield the expected angular velocity of only that difference
  current_pose.set_orientation(Eigen::Quaterniond(1, 1, 0, 0).normalized() * ring.get_rotation_offset());
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), -M_PI_2 * ring.get_angular_gain(), tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);

  // rotate the center plane in the base frame, and set the current position to have the same relative
  // offset that gives a zero command (no rotation offset)
  center.set_orientation(Eigen::Quaterniond::UnitRandom());
  ring.set_center(center);
  current_pose = state_representation::CartesianPose("B", Eigen::Vector3d(radius, 0, 0), "A");
  current_pose = center * current_pose;
  ring.set_rotation_offset(Eigen::Quaterniond::Identity());
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // for any rotation offset in a rotated center plane, the output will
  // still be zero if the current position and orientation matches the rotation offset
  rotation = Eigen::Quaterniond::UnitRandom();
  ring.set_rotation_offset(rotation);
  current_pose = state_representation::CartesianPose("B",
                                                     Eigen::Vector3d(radius, 0, 0),
                                                     ring.get_rotation_offset(),
                                                     "A");
  current_pose = center * current_pose;
  twist = ring.evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // any additional orientation in the ring frame on top of the rotation offset
  // should give the same local command, regardless of the center frame
  current_pose = state_representation::CartesianPose("B",
                                                     Eigen::Vector3d(radius, 0, 0),
                                                     Eigen::Quaterniond(1, 1, 0, 0).normalized()  * ring.get_rotation_offset(),
                                                     "A");
  current_pose = center * current_pose;
  twist = ring.evaluate(current_pose);

  // check the twist in the local frame
  twist = state_representation::CartesianState(center).inverse() * twist;
  EXPECT_NEAR(twist.get_angular_velocity().x(), -M_PI_2 * ring.get_angular_gain(), tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);
}

TEST_F(RingDSTest, BaseFrameBehaviours) {
  auto AinB = state_representation::CartesianState::Random("A", "B");
  dynamical_systems::Ring ring(AinB);

  // setting the center through the constructor should also set the base frame (as Identity frame)
  EXPECT_STREQ(ring.get_center().get_name().c_str(), "A");
  EXPECT_STREQ(ring.get_center().get_reference_frame().c_str(), "B");
  EXPECT_STREQ(ring.get_base_frame().get_name().c_str(), "B");
  EXPECT_STREQ(ring.get_base_frame().get_reference_frame().c_str(), "B");
  EXPECT_NEAR(ring.get_base_frame().get_pose().norm(), 1, tol);

  // setting the center should fail if it is incompatible with the base frame
  auto CinD = state_representation::CartesianState::Random("C", "D");
  EXPECT_THROW(ring.set_center(CinD), state_representation::exceptions::IncompatibleReferenceFramesException);

  // updating the base frame should "move" the center frame but not change its magnitude
  auto centerNorm = ring.get_center().get_pose().norm();
  ring.set_base_frame(CinD);

  EXPECT_STREQ(ring.get_center().get_name().c_str(), "A");
  EXPECT_STREQ(ring.get_center().get_reference_frame().c_str(), "C");
  EXPECT_STREQ(ring.get_base_frame().get_name().c_str(), "C");
  EXPECT_STREQ(ring.get_base_frame().get_reference_frame().c_str(), "D");
  EXPECT_NEAR(ring.get_center().get_pose().norm(), centerNorm, tol);

  // setting the center should succeed if it is expressed relative to the base frame
  auto BinC = state_representation::CartesianState::Random("B", "C");
  EXPECT_NO_THROW(ring.set_center(BinC));
  EXPECT_STREQ(ring.get_center().get_name().c_str(), "B");
  EXPECT_STREQ(ring.get_center().get_reference_frame().c_str(), "C");

  // setting the center should also succeed if it shares the same reference frame as the base frame
  auto BinD = state_representation::CartesianState::Random("B", "D");
  EXPECT_NO_THROW(ring.set_center(BinD));
  EXPECT_STREQ(ring.get_center().get_name().c_str(), "B");
  // the reference frame is still C, not D, because the center is internally represented relative to the base frame (C)
  EXPECT_STREQ(ring.get_center().get_reference_frame().c_str(), "C");
}

TEST_F(RingDSTest, SettersAndGetters) {
  dynamical_systems::Ring ring(center);

  auto pose = state_representation::CartesianState::Random("C");
  ring.set_center(pose);
  auto pose2 = ring.get_center();
  EXPECT_STREQ(pose.get_name().c_str(), pose2.get_name().c_str());
  EXPECT_STREQ(pose.get_reference_frame().c_str(), pose2.get_reference_frame().c_str());
  EXPECT_NEAR(pose.get_pose().norm(), pose2.get_pose().norm(), tol);

  // all other setters should store the value
  ring.set_rotation_offset(Eigen::Quaterniond(1, 2, 3, 4).normalized());
  EXPECT_NEAR(ring.get_rotation_offset().angularDistance(Eigen::Quaterniond(1, 2, 3, 4).normalized()), 0, tol);

  ring.set_radius(1);
  EXPECT_NEAR(ring.get_radius(), 1, tol);

  ring.set_width(2);
  EXPECT_NEAR(ring.get_width(), 2, tol);

  ring.set_speed(3);
  EXPECT_NEAR(ring.get_speed(), 3, tol);

  ring.set_field_strength(4);
  EXPECT_NEAR(ring.get_field_strength(), 4, tol);

  ring.set_normal_gain(5);
  EXPECT_NEAR(ring.get_normal_gain(), 5, tol);

  ring.set_angular_gain(6);
  EXPECT_NEAR(ring.get_angular_gain(), 6, tol);
}