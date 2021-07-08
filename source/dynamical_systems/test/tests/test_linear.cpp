#include <vector>
#include <gtest/gtest.h>
#include "dynamical_systems/Linear.hpp"
#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

using namespace state_representation;
using namespace dynamical_systems;
using namespace std::literals::chrono_literals;

class LinearDSTest : public testing::Test {
protected:
  void SetUp() override {
    current_pose = CartesianPose("A", 10 * Eigen::Vector3d::Random());
    target_pose = CartesianPose("B", 10 * Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  }
  void print_current_and_target_pose() {
    std::cout << current_pose << std::endl;
    std::cout << target_pose << std::endl;
    std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;
  }

  CartesianPose current_pose;
  CartesianPose target_pose;
  unsigned int nb_steps = 100;
  std::chrono::milliseconds dt = 100ms;
  double linear_tol = 1e-3;
  double angular_tol = 1e-3;
};

TEST_F(LinearDSTest, EmptyConstructorCartesianState) {
  // construct empty cartesian state DS
  CartesianPose attractor = CartesianPose::Identity("CAttractor", "A");
  Linear<CartesianState> ds;
  // base frame and attractor should be empty
  EXPECT_TRUE(ds.get_attractor().is_empty());
  EXPECT_TRUE(ds.get_base_frame().is_empty());
  ds.set_attractor(attractor);
  EXPECT_FALSE(ds.get_attractor().is_empty());
  EXPECT_FALSE(ds.get_base_frame().is_empty());
  // when attractor was set without a base frame, expect base frame to be identity with name / reference_frame of attractor
  EXPECT_EQ(ds.get_base_frame().get_name(), attractor.get_reference_frame());
  EXPECT_EQ(ds.get_base_frame().get_reference_frame(), attractor.get_reference_frame());
  EXPECT_EQ(ds.get_base_frame().get_transformation_matrix(), Eigen::Matrix4d::Identity());

  ds = Linear<CartesianState>();
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

  ds.set_attractor(attractor);
  EXPECT_TRUE(ds.is_compatible(state1));
  EXPECT_FALSE(ds.is_compatible(state2));
  EXPECT_TRUE(ds.is_compatible(state3));
  EXPECT_TRUE(ds.is_compatible(state4));
}

TEST_F(LinearDSTest, EmptyConstructorJointState) {
  // construct empty joint state DS
  Linear<JointState> ds;
  JointState attractor = JointState::Zero("robot", 3);

  EXPECT_TRUE(ds.get_attractor().is_empty());
  EXPECT_TRUE(ds.get_base_frame().is_empty());
  ds.set_attractor(attractor);
  EXPECT_FALSE(ds.get_attractor().is_empty());
  EXPECT_FALSE(ds.get_base_frame().is_empty());
  // when attractor was set without a base frame, expect base frame to be zero with name of attractor
  EXPECT_EQ(ds.get_base_frame().get_name(), attractor.get_name());
  EXPECT_EQ(ds.get_base_frame().get_names(), attractor.get_names());
  EXPECT_EQ(ds.get_base_frame().data(), Eigen::VectorXd::Zero(attractor.get_size() * 4));

  ds = Linear<JointState>();
  JointState state1 = JointState::Zero("robot", 2);
  JointState state2 = JointState::Zero("robot", 3);
  JointState state3 = JointState::Zero("dummy", 2);
  // if no base frame is set, an exception is thrown
  EXPECT_THROW(ds.evaluate(state1), dynamical_systems::exceptions::EmptyBaseFrameException);
  ds.set_base_frame(state1);
  // if joint state is incompatible (here with empty base frame), an exception is thrown
  EXPECT_THROW(ds.evaluate(state2), state_representation::exceptions::IncompatibleReferenceFramesException);
  // if no attractor is set, an exception is thrown
  EXPECT_THROW(ds.evaluate(state1), dynamical_systems::exceptions::EmptyAttractorException);
  // if attractor with incompatible robot names should be set, an exception is thrown
  EXPECT_THROW(ds.set_attractor(state3), state_representation::exceptions::IncompatibleReferenceFramesException);
  // if attractor with incompatible joint names should be set, an exception is thrown
  EXPECT_THROW(ds.set_attractor(attractor), state_representation::exceptions::IncompatibleReferenceFramesException);
  ds.set_base_frame(state2);
  ds.set_attractor(attractor);
  EXPECT_TRUE(ds.is_compatible(state2));
  EXPECT_FALSE(ds.is_compatible(state3));
  EXPECT_NO_THROW(ds.evaluate(state2));
  EXPECT_THROW(ds.evaluate(state3), state_representation::exceptions::IncompatibleReferenceFramesException);
}

TEST_F(LinearDSTest, IsCompatible) {
  CartesianState state1("C", "A");
  CartesianState state2("C", "B");
  CartesianState state3("C", "D");

  CartesianState attractor_frame = CartesianState::Identity("CAttractor", "A");
  Linear<CartesianState> ds(attractor_frame);
  EXPECT_TRUE(ds.is_compatible(state1));
  EXPECT_FALSE(ds.is_compatible(state2));
  EXPECT_FALSE(ds.is_compatible(state3));

  // change the base frame
  CartesianState base_frame = CartesianState::Identity("A", "B");
  ds.set_base_frame(base_frame);
  EXPECT_TRUE(ds.is_compatible(state1));
  EXPECT_TRUE(ds.is_compatible(state2));
  EXPECT_FALSE(ds.is_compatible(state3));
}

TEST_F(LinearDSTest, PositionOnly) {
  current_pose.set_orientation(Eigen::Quaterniond::Identity());
  target_pose.set_orientation(Eigen::Quaterniond::Identity());
  Linear<CartesianState> linearDS(target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::POSITION), 0, linear_tol);
}

TEST_F(LinearDSTest, OrientationOnly) {
  current_pose.set_position(Eigen::Vector3d::Zero());
  target_pose.set_position(Eigen::Vector3d::Zero());

  Linear<CartesianState> linearDS(target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::ORIENTATION), 0, angular_tol);
}

TEST_F(LinearDSTest, PositionAndOrientation) {
  Linear<CartesianState> linearDS(target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::POSITION), 0, linear_tol);
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::ORIENTATION), 0, angular_tol);
}

TEST_F(LinearDSTest, JointState) {
  auto current_positions = JointPositions::Zero("robot", 3);
  auto target_positions = JointPositions::Random("robot", 3);
  Linear<JointState> linearDS(target_positions);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    JointVelocities velocities = linearDS.evaluate(current_positions);
    current_positions += dt * velocities;
  }
  EXPECT_NEAR(current_positions.dist(target_positions), 0, linear_tol);
}

TEST_F(LinearDSTest, NonUniformGainValues) {
  std::vector<double> gains(6, 0);
  Linear<CartesianState> linearDS(target_pose, gains);

  // expect no twist when DS gains are all zero
  CartesianTwist twist = linearDS.evaluate(current_pose);
  EXPECT_NEAR(twist.get_linear_velocity().norm(), 0, linear_tol);
  EXPECT_NEAR(twist.get_angular_velocity().norm(), 0, angular_tol);

  // check that the returned twist only applies in the dimension with a non-zero gain
  for (unsigned int i = 0; i < gains.size(); ++i) {
    gains.at(i) = 1;
    linearDS.set_gain(gains);

    twist = linearDS.evaluate(current_pose);

    auto vec = twist.data();
    for (unsigned int ax = 0; ax < vec.size(); ++ax) {
      if (ax == i) {
        EXPECT_TRUE(abs(vec(ax)) > 0);
      } else {
        EXPECT_NEAR(vec(ax), 0, 1e-9);
      }
    }

    gains.at(i) = 0;
  }
}

TEST(LinearDSTestFrames, FixedReferenceFrames) {
  auto BinA = CartesianState::Identity("B", "A");
  auto CinA = CartesianState::Identity("C", "A");
  auto CinB = CartesianState::Identity("C", "B");
  BinA.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  CinA.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  CinB.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());


  // initialise the linearDS with attractor at B in reference frame A
  Linear<CartesianState> linearDS(BinA);

  // evaluating a current pose B in reference frame A should give zero twist (coincident with attractor)
  CartesianTwist twist = linearDS.evaluate(BinA);
  EXPECT_NEAR(twist.data().norm(), 0, 1e-5);

  // evaluating pose C in frame A should give some non-zero twist
  twist = linearDS.evaluate(CinA);
  EXPECT_GE(twist.data().norm(), 1e-5);

  // evaluating a state which does not match the DS base frame A should give an error
  EXPECT_THROW(linearDS.evaluate(CinB), state_representation::exceptions::IncompatibleReferenceFramesException);

  // an inverse state (A expressed in C) should also give an error,
  // since the reference frame does not explicitly match the DS base frame
  EXPECT_THROW(linearDS.evaluate(CinA.inverse()),
               state_representation::exceptions::IncompatibleReferenceFramesException);
}

TEST(LinearDSTestFrames, UpdateBaseReferenceFrames) {
  auto BinA = CartesianState::Random("B", "A");

  // initialise the linearDS with attractor at B in reference frame A
  Linear<CartesianState> linearDS(BinA);

  // the base frame of the default constructed DS should be an identity frame
  // with the same name as the attractor reference frame
  auto base = linearDS.get_base_frame();
  EXPECT_STREQ(base.get_name().c_str(), BinA.get_reference_frame().c_str());
  EXPECT_STREQ(base.get_name().c_str(), base.get_reference_frame().c_str());
  EXPECT_NEAR(base.get_pose().norm(), 1, 1e-5);

  auto AinWorld = CartesianState::Random("A", "world");
  linearDS.set_base_frame(AinWorld);

  // check the setter
  base = linearDS.get_base_frame();
  EXPECT_STREQ(base.get_name().c_str(), AinWorld.get_name().c_str());
  EXPECT_STREQ(base.get_reference_frame().c_str(), AinWorld.get_reference_frame().c_str());
  EXPECT_NEAR(base.get_pose().norm(), AinWorld.get_pose().norm(), 1e-5);

  // evaluating a current pose B in reference frame A should give zero twist (coincident with attractor)
  CartesianTwist twist = linearDS.evaluate(BinA);
  EXPECT_STREQ(twist.get_name().c_str(), BinA.get_name().c_str());
  EXPECT_STREQ(twist.get_reference_frame().c_str(), BinA.get_reference_frame().c_str());
  EXPECT_NEAR(twist.data().norm(), 0, 1e-5);

  // evaluating a current pose B in reference frame world should give zero twist (coincident with attractor)
  // PLUS the twist of A in world, expressed as a twist B in world
  auto BinWorld = AinWorld * BinA;
  twist = linearDS.evaluate(BinWorld);
  EXPECT_STREQ(twist.get_name().c_str(), BinWorld.get_name().c_str());
  EXPECT_STREQ(twist.get_reference_frame().c_str(), BinWorld.get_reference_frame().c_str());
  EXPECT_NEAR(twist.data().norm(), 0 + AinWorld.get_twist().norm(), 1e-5);

  // since the base frame is set to frame A in frame world, evaluating a state with a reference frame that is neither
  // should give an error
  EXPECT_THROW(linearDS.evaluate(BinWorld.inverse()),
               state_representation::exceptions::IncompatibleReferenceFramesException);
}

TEST(LinearDSTestFrames, StackedMovingReferenceFrames) {
  auto AinWorld = CartesianState::Random("A", "world");
  auto BinA = CartesianState::Random("B", "A");

  auto CinA = CartesianState::Identity("C", "A");
  CinA.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

  // initialise the linearDS with attractor at B in reference frame A
  Linear<CartesianState> linearDS(BinA);

  // evaluate the twist for a fixed state C in reference frame A
  CartesianTwist twist = linearDS.evaluate(CinA);

  // the twist should be the same for a moving state C in reference frame A
  // (the DS should always give the velocity from position, not from velocity)
  CinA.set_linear_velocity(Eigen::Vector3d::Random());
  CinA.set_angular_velocity(Eigen::Vector3d::Random());
  CartesianTwist twist2 = linearDS.evaluate(CinA);

  EXPECT_NEAR(twist.data().norm(), twist2.data().norm(), 1e-5);

  // since reference frame A is moving with respect to the world, and DS gives a twist in reference frame A,
  // the operation to express DS twist in the world frame should be valid.
  ASSERT_NO_THROW(twist = AinWorld * linearDS.evaluate(CinA));

  // the same operation can be done by setting the base frame of the DS
  linearDS.set_base_frame(AinWorld);
  auto CinWorld = AinWorld * CinA;
  ASSERT_NO_THROW(twist2 = linearDS.evaluate(CinWorld));

  // both methods should give the twist in the world frame
  EXPECT_STREQ(twist.get_reference_frame().c_str(), AinWorld.get_reference_frame().c_str());
  EXPECT_STREQ(twist.get_reference_frame().c_str(), twist2.get_reference_frame().c_str());
  EXPECT_NEAR(twist.data().norm(), twist2.data().norm(), 1e-5);

}

TEST(LinearDSTestFrames, UpdateAttractorFrame) {
  CartesianState A, B, C, D;
  A = CartesianState::Random("A", "world");
  B = CartesianState::Random("B", "world");
  C = CartesianState::Random("C", "robot");
  D = CartesianState::Random("D", "robot");

  Linear<CartesianState> linearDS(A);

  // state being evaluated must match the DS base frame, which is by default the attractor reference frame
  EXPECT_NO_THROW(linearDS.evaluate(B));
  EXPECT_THROW(linearDS.evaluate(D), state_representation::exceptions::IncompatibleReferenceFramesException);

  // setting the attractor to another point in the same base frame should be fine,
  // but setting it with a different base frame should give an error
  EXPECT_NO_THROW(linearDS.set_attractor(B));
  EXPECT_THROW(linearDS.set_attractor(C), state_representation::exceptions::IncompatibleReferenceFramesException);

  // after updating the base frame, the attractor reference frame should also be updated
  linearDS.set_base_frame(CartesianState::Identity(C.get_reference_frame(), C.get_reference_frame()));
  EXPECT_STREQ(linearDS.get_attractor().get_reference_frame().c_str(), C.get_reference_frame().c_str());

  // with the new base frame, setting the attractor should succeed / fail accordingly
  EXPECT_THROW(linearDS.set_attractor(B), state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(linearDS.set_attractor(C));

  // now the evaluation should also succeed when matching the updated base frame
  EXPECT_THROW(linearDS.evaluate(B), state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(linearDS.evaluate(D));
}