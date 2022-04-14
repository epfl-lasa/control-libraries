#include <gtest/gtest.h>

#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace dynamical_systems;
using namespace state_representation;
using namespace std::literals::chrono_literals;

class RingDSTest : public ::testing::Test {
protected:
  RingDSTest() {
    ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::RING);
    center = CartesianPose::Identity("A");
    current_pose = CartesianPose("B", radius * Eigen::Vector3d::Random());
  }

  std::shared_ptr<IDynamicalSystem<CartesianState>> ds;
  CartesianPose current_pose;
  CartesianPose center;
  std::vector<double> vlim = {10, 10, 0.001, 0.001};
  double radius = 10;
  double width = 1;
  double speed = 1;
  double field_strength = 0.5;
  unsigned int nb_steps = 1000;
  std::chrono::milliseconds dt = 10ms;
  double tol = 1e-3;
};

TEST_F(RingDSTest, EmptyConstructor) {
  CartesianPose center = CartesianPose::Identity("CAttractor", "A");
  // base frame and attractor should be empty
  EXPECT_TRUE(ds->get_parameter_value<CartesianPose>("center").is_empty());
  ds->set_parameter_value("center", center);
  EXPECT_FALSE(ds->get_parameter_value<CartesianPose>("center").is_empty());
  EXPECT_FALSE(ds->get_base_frame().is_empty());
  // when attractor was set without a base frame, expect base frame to be identity with name / reference_frame of attractor
  EXPECT_EQ(ds->get_base_frame().get_name(), center.get_reference_frame());
  EXPECT_EQ(ds->get_base_frame().get_reference_frame(), center.get_reference_frame());
  EXPECT_EQ(ds->get_base_frame().get_transformation_matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(RingDSTest, EvaluateComptability) {
  CartesianPose center = CartesianPose::Identity("CAttractor", "A");
  CartesianState state1 = CartesianState::Identity("B", "A");
  CartesianState state2("D", "C");
  CartesianState state3("C", "A");
  CartesianState state4("C", "B");
  // if no base frame is set, an exception is thrown
  EXPECT_THROW(ds->evaluate(state1), dynamical_systems::exceptions::EmptyBaseFrameException);
  ds->set_base_frame(state1);
  // if cartesian state is incompatible, an exception is thrown
  EXPECT_THROW(ds->evaluate(state2), state_representation::exceptions::IncompatibleReferenceFramesException);
  // if cartesian state needs to be transformed to other frame first and is empty, an exception is thrown
  EXPECT_THROW(ds->evaluate(state3), state_representation::exceptions::EmptyStateException);
  // if no attractor is set, an exception is thrown
  EXPECT_THROW(ds->evaluate(state4), dynamical_systems::exceptions::EmptyAttractorException);

  ds->set_parameter_value("center", center);
  EXPECT_TRUE(ds->is_compatible(state1));
  EXPECT_FALSE(ds->is_compatible(state2));
  EXPECT_TRUE(ds->is_compatible(state3));
  EXPECT_TRUE(ds->is_compatible(state4));
}

TEST_F(RingDSTest, PointsOnRadius) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);
  ds->set_parameter_value("width", width);
  ds->set_parameter_value("speed", speed);
  CartesianTwist twist;

  // zero output at center
  current_pose = center;
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.data().norm(), 0, tol);

  current_pose.set_position(radius, 0, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), speed, tol);

  current_pose.set_position(0, radius, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), -speed, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(-radius, 0, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), -speed, tol);

  current_pose.set_position(0, -radius, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), speed, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(radius, 0, 1);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().z(), -1, tol);
}

TEST_F(RingDSTest, PointsNearRadius) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);
  ds->set_parameter_value("width", width);
  ds->set_parameter_value("speed", speed);
  ds->set_parameter_value("field_strength", field_strength);
  CartesianTwist twist;

  current_pose.set_position(radius + width, 0, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), -speed * field_strength, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(radius - width, 0, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), speed * field_strength, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);

  current_pose.set_position(0, radius + width, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), -speed * field_strength, tol);

  current_pose.set_position(0, radius - width, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), speed * field_strength, tol);

  current_pose.set_position(radius + (width * 2), 0, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().x(), -speed * field_strength, tol);
  EXPECT_NEAR(twist.get_linear_velocity().y(), 0, tol);
}

TEST_F(RingDSTest, BehaviourNearBoundaryAtHighSpeeds) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", 1.0);
  ds->set_parameter_value("width", 0.1);
  ds->set_parameter_value("speed", 10.0);
  ds->set_parameter_value("field_strength", 2.0);
  CartesianTwist twist;

  current_pose.set_position(0, radius + 1.001 * width, 0);
  twist = ds->evaluate(current_pose);
  double angular_speed = twist.get_angular_velocity().norm();
  current_pose.set_position(0, radius + 1.000 * width, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), angular_speed, 0.1);
  current_pose.set_position(0, radius + 0.999 * width, 0);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), angular_speed, 0.1);
}

TEST_F(RingDSTest, ConvergenceOnRadius) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    twist.clamp(vlim[0], vlim[1], vlim[2], vlim[3]);
    current_pose += dt * twist;
  }

  EXPECT_NEAR((current_pose.get_position() - center.get_position()).norm(), radius, tol);
}

TEST_F(RingDSTest, ConvergenceOnRadiusRandomCenter) {
  center.set_position(Eigen::Vector3d::Random());
  center.set_orientation(Eigen::Quaterniond::UnitRandom());
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    twist.clamp(vlim[0], vlim[1], vlim[2], vlim[3]);
    current_pose += dt * twist;
  }

  EXPECT_NEAR((current_pose.get_position() - center.get_position()).norm(), radius, tol);
}

TEST_F(RingDSTest, ZeroNormalGain) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("normal_gain", 0.0);

  double startingHeight = current_pose.get_position().z();
  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    twist.clamp(vlim[0], vlim[1], vlim[2], vlim[3]);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.get_position().z(), startingHeight, tol);
}

TEST_F(RingDSTest, OrientationAroundCircle) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);
  ds->set_parameter_value("width", width);
  ds->set_parameter_value("speed", 0.0);
  CartesianTwist twist;

  // at the position {radius, 0, 0}, the orientation attractor is by default null,
  // so there should be zero angular velocity if the current pose orientation is also null
  current_pose.set_position(radius, 0, 0);
  current_pose.set_orientation(Eigen::Quaterniond::Identity());
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // at the position {0, radius, 0}, the default rotation around Z is pi/2
  current_pose.set_position(0, radius, 0);
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 0, 1));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // at the position {0, radius, 0}, the default rotation around Z is pi
  current_pose.set_position(-radius, 0, 0);
  current_pose.set_orientation(Eigen::Quaterniond(0, 0, 0, 1));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // at the position {0, radius, 0}, the default rotation around Z is 3 * pi/2
  current_pose.set_position(0, -radius, 0);
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 0, -1));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);
}

TEST_F(RingDSTest, OrientationRestitutionAtZeroAngle) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);
  ds->set_parameter_value("width", width);
  ds->set_parameter_value("speed", 0.0);
  CartesianTwist twist;

  // at the position {radius, 0, 0}, the orientation attractor is by default null (angle around circle is 0)
  current_pose.set_position(radius, 0, 0);

  // rotate the current pose around local X
  current_pose.set_orientation(Eigen::Quaterniond(1, 1, 0, 0));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), -M_PI_2 * ds->get_parameter_value<double>("angular_gain"), tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);

  // rotate the current pose around local Y
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 1, 0));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), -M_PI_2 * ds->get_parameter_value<double>("angular_gain"), tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);

  // rotate the current pose around local Z
  current_pose.set_orientation(Eigen::Quaterniond(1, 0, 0, 1));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), -M_PI_2 * ds->get_parameter_value<double>("angular_gain"), tol);
}

TEST_F(RingDSTest, OrientationRotationOffset) {
  ds->set_parameter_value("center", center);
  ds->set_parameter_value("radius", radius);
  ds->set_parameter_value("width", width);
  ds->set_parameter_value("speed", 0.0);
  CartesianTwist twist;

  current_pose.set_position(radius, 0, 0);

  Eigen::Quaterniond rotation = Eigen::Quaterniond(1, 0, 1, 0).normalized(); // Eigen::Quaterniond::UnitRandom();

  // if the rotation offset is the same as the current orientation, the angular velocity at
  // position {radius, 0, 0} is always zero
  current_pose.set_orientation(rotation);
  ds->set_parameter_value("rotation_offset", current_pose);
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // now if the current pose has some orientation relative to the rotation offset,
  // it will yield the expected angular velocity of only that difference
  current_pose.set_orientation(
      Eigen::Quaterniond(1, 1, 0, 0).normalized()
          * ds->get_parameter_value<CartesianPose>("rotation_offset").get_orientation());
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().x(), -M_PI_2 * ds->get_parameter_value<double>("angular_gain"), tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);

  // rotate the center plane in the base frame, and set the current position to have the same relative
  // offset that gives a zero command (no rotation offset)
  center.set_orientation(Eigen::Quaterniond::UnitRandom());
  ds->set_parameter_value("center", center);
  current_pose = CartesianPose("B", Eigen::Vector3d(radius, 0, 0), "A");
  current_pose = center * current_pose;
  ds->set_parameter_value("rotation_offset", CartesianPose::Identity("offset"));
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // for any rotation offset in a rotated center plane, the output will
  // still be zero if the current position and orientation matches the rotation offset
  rotation = Eigen::Quaterniond::UnitRandom();
  ds->set_parameter_value("rotation_offset", CartesianPose("offset", rotation));
  current_pose = CartesianPose(
      "B", Eigen::Vector3d(radius, 0, 0), ds->get_parameter_value<CartesianPose>("rotation_offset").get_orientation(),
      "A"
  );
  current_pose = center * current_pose;
  twist = ds->evaluate(current_pose);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, tol);

  // any additional orientation in the ring frame on top of the rotation offset
  // should give the same local command, regardless of the center frame
  current_pose = CartesianPose(
      "B", Eigen::Vector3d(radius, 0, 0), Eigen::Quaterniond(1, 1, 0, 0).normalized()
          * ds->get_parameter_value<CartesianPose>("rotation_offset").get_orientation(), "A"
  );
  current_pose = center * current_pose;
  twist = ds->evaluate(current_pose);

  // check the twist in the local frame
  twist = CartesianState(center).inverse() * twist;
  EXPECT_NEAR(twist.get_angular_velocity().x(), -M_PI_2 * ds->get_parameter_value<double>("angular_gain"), tol);
  EXPECT_NEAR(twist.get_angular_velocity().y(), 0, tol);
  EXPECT_NEAR(twist.get_angular_velocity().z(), 0, tol);
}

TEST_F(RingDSTest, BaseFrameBehaviours) {
  auto AinB = CartesianPose::Random("A", "B");
  ds->set_parameter_value("center", AinB);

  // setting the center through the constructor should also set the base frame (as Identity frame)
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_name().c_str(), "A");
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_reference_frame().c_str(), "B");
  EXPECT_STREQ(ds->get_base_frame().get_name().c_str(), "B");
  EXPECT_STREQ(ds->get_base_frame().get_reference_frame().c_str(), "B");
  EXPECT_NEAR(ds->get_base_frame().get_pose().norm(), 1, tol);

  // setting the center should fail if it is incompatible with the base frame
  auto CinD = CartesianPose::Random("C", "D");
  EXPECT_THROW(ds->set_parameter_value("center", CartesianPose(CinD)),
               state_representation::exceptions::IncompatibleReferenceFramesException);

  // updating the base frame should "move" the center frame but not change its magnitude
  auto centerNorm = ds->get_parameter_value<CartesianPose>("center").get_pose().norm();
  ds->set_base_frame(CinD);

  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_name().c_str(), "A");
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_reference_frame().c_str(), "C");
  EXPECT_STREQ(ds->get_base_frame().get_name().c_str(), "C");
  EXPECT_STREQ(ds->get_base_frame().get_reference_frame().c_str(), "D");
  EXPECT_NEAR(ds->get_parameter_value<CartesianPose>("center").get_pose().norm(), centerNorm, tol);

  // setting the center should succeed if it is expressed relative to the base frame
  auto BinC = CartesianPose::Random("B", "C");
  EXPECT_NO_THROW(ds->set_parameter_value("center", BinC));
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_name().c_str(), "B");
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_reference_frame().c_str(), "C");

  // setting the center should also succeed if it shares the same reference frame as the base frame
  auto BinD = CartesianPose::Random("B", "D");
  EXPECT_NO_THROW(ds->set_parameter_value("center", BinD));
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_name().c_str(), "B");
  // the reference frame is still C, not D, because the center is internally represented relative to the base frame (C)
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("center").get_reference_frame().c_str(), "C");
}

TEST_F(RingDSTest, SettersAndGetters) {
  ds->set_parameter_value("center", center);

  auto pose = CartesianState::Random("C");
  ds->set_parameter_value("center", CartesianPose(pose));
  auto pose2 = ds->get_parameter_value<CartesianPose>("center");
  EXPECT_STREQ(pose.get_name().c_str(), pose2.get_name().c_str());
  EXPECT_STREQ(pose.get_reference_frame().c_str(), pose2.get_reference_frame().c_str());
  EXPECT_NEAR(pose.get_pose().norm(), pose2.get_pose().norm(), tol);

  // all other setters should store the value
  ds->set_parameter_value(
      "rotation_offset", CartesianPose("offset", Eigen::Quaterniond(1, 2, 3, 4).normalized()));
  EXPECT_NEAR(ds->get_parameter_value<CartesianPose>("rotation_offset").get_orientation().angularDistance(
      Eigen::Quaterniond(1, 2, 3, 4).normalized()), 0, tol);

  ds->set_parameter_value("radius", 1.0);
  EXPECT_NEAR(ds->get_parameter_value<double>("radius"), 1.0, tol);

  ds->set_parameter_value("width", 2.0);
  EXPECT_NEAR(ds->get_parameter_value<double>("width"), 2.0, tol);

  ds->set_parameter_value("speed", 3.0);
  EXPECT_NEAR(ds->get_parameter_value<double>("speed"), 3.0, tol);

  ds->set_parameter_value("field_strength", 4.0);
  EXPECT_NEAR(ds->get_parameter_value<double>("field_strength"), 4.0, tol);

  ds->set_parameter_value("normal_gain", 5.0);
  EXPECT_NEAR(ds->get_parameter_value<double>("normal_gain"), 5.0, tol);

  ds->set_parameter_value("angular_gain", 6.0);
  EXPECT_NEAR(ds->get_parameter_value<double>("angular_gain"), 6.0, tol);
}
