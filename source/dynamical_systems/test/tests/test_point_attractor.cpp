#include <vector>
#include <gtest/gtest.h>
#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"


using namespace state_representation;
using namespace dynamical_systems;
using namespace std::literals::chrono_literals;

class PointAttractorTest : public testing::Test {
protected:
  void SetUp() override {
    current_pose = CartesianPose("A", 10 * Eigen::Vector3d::Random());
    target_pose = CartesianPose("B", 10 * Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
    ds = DynamicalSystemFactory<CartesianState>::create_dynamical_system(
        DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
    );
  }
  void print_current_and_target_pose() {
    std::cout << current_pose << std::endl;
    std::cout << target_pose << std::endl;
    std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;
  }

  std::shared_ptr<IDynamicalSystem<CartesianState>> ds;
  CartesianPose current_pose;
  CartesianPose target_pose;
  unsigned int nb_steps = 100;
  std::chrono::milliseconds dt = 100ms;
  double linear_tol = 1e-3;
  double angular_tol = 1e-3;
};

TEST_F(PointAttractorTest, EmptyConstructorCartesianState) {
  // construct empty cartesian state DS
  CartesianPose attractor = CartesianPose::Identity("CAttractor", "A");

  // base frame and attractor should be empty
  EXPECT_TRUE(ds->get_parameter_value<CartesianPose>("attractor").is_empty());
  EXPECT_TRUE(ds->get_base_frame().is_empty());
  ds->set_parameter_value("attractor", attractor);
  EXPECT_FALSE(ds->get_parameter_value<CartesianPose>("attractor").is_empty());
  EXPECT_FALSE(ds->get_base_frame().is_empty());
  // when attractor was set without a base frame, expect base frame to be identity with name / reference_frame of attractor
  EXPECT_EQ(ds->get_base_frame().get_name(), attractor.get_reference_frame());
  EXPECT_EQ(ds->get_base_frame().get_reference_frame(), attractor.get_reference_frame());
  EXPECT_EQ(ds->get_base_frame().get_transformation_matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(PointAttractorTest, EmptyIsCompatible) {
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

  ds->set_parameter_value("attractor", CartesianPose::Identity("CAttractor", "A"));
  EXPECT_TRUE(ds->is_compatible(state1));
  EXPECT_FALSE(ds->is_compatible(state2));
  EXPECT_TRUE(ds->is_compatible(state3));
  EXPECT_TRUE(ds->is_compatible(state4));
}

TEST_F(PointAttractorTest, IsCompatible) {
  CartesianState state1("C", "A");
  CartesianState state2("C", "B");
  CartesianState state3("C", "D");

  CartesianPose attractor_frame = CartesianPose::Identity("CAttractor", "A");
  ds->set_parameter_value("attractor", attractor_frame);
  EXPECT_TRUE(ds->is_compatible(state1));
  EXPECT_FALSE(ds->is_compatible(state2));
  EXPECT_FALSE(ds->is_compatible(state3));

  // change the base frame
  CartesianState base_frame = CartesianState::Identity("A", "B");
  ds->set_base_frame(base_frame);
  EXPECT_TRUE(ds->is_compatible(state1));
  EXPECT_TRUE(ds->is_compatible(state2));
  EXPECT_FALSE(ds->is_compatible(state3));
}

TEST_F(PointAttractorTest, PositionOnly) {
  current_pose.set_orientation(Eigen::Quaterniond::Identity());
  target_pose.set_orientation(Eigen::Quaterniond::Identity());
  ds->set_parameter_value("attractor", target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::POSITION), 0, linear_tol);
}

TEST_F(PointAttractorTest, OrientationOnly) {
  current_pose.set_position(Eigen::Vector3d::Zero());
  target_pose.set_position(Eigen::Vector3d::Zero());
  ds->set_parameter_value("attractor", target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::ORIENTATION), 0, angular_tol);
}

TEST_F(PointAttractorTest, PositionAndOrientation) {
  ds->set_parameter_value("attractor", target_pose);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    current_pose += dt * twist;
  }
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::POSITION), 0, linear_tol);
  EXPECT_NEAR(current_pose.dist(target_pose, CartesianStateVariable::ORIENTATION), 0, angular_tol);
}

TEST_F(PointAttractorTest, FixedReferenceFrames) {
  auto BinA = CartesianState::Identity("B", "A");
  auto CinA = CartesianState::Identity("C", "A");
  auto CinB = CartesianState::Identity("C", "B");
  BinA.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  CinA.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  CinB.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

  ds->set_parameter_value("attractor", BinA);

  // evaluating a current pose B in reference frame A should give zero twist (coincident with attractor)
  CartesianTwist twist = ds->evaluate(BinA);
  EXPECT_NEAR(twist.data().norm(), 0, 1e-5);

  // evaluating pose C in frame A should give some non-zero twist
  twist = ds->evaluate(CinA);
  EXPECT_GE(twist.data().norm(), 1e-5);

  // evaluating a state which does not match the DS base frame A should give an error
  EXPECT_THROW(ds->evaluate(CinB), state_representation::exceptions::IncompatibleReferenceFramesException);

  // an inverse state (A expressed in C) should also give an error,
  // since the reference frame does not explicitly match the DS base frame
  EXPECT_THROW(ds->evaluate(CinA.inverse()), state_representation::exceptions::IncompatibleReferenceFramesException);
}

TEST_F(PointAttractorTest, UpdateBaseReferenceFrames) {
  auto BinA = CartesianState::Random("B", "A");
  ds->set_parameter_value("attractor", BinA);

  // the base frame of the default constructed DS should be an identity frame
  // with the same name as the attractor reference frame
  auto base = ds->get_base_frame();
  EXPECT_STREQ(base.get_name().c_str(), BinA.get_reference_frame().c_str());
  EXPECT_STREQ(base.get_name().c_str(), base.get_reference_frame().c_str());
  EXPECT_NEAR(base.get_pose().norm(), 1, 1e-5);

  auto AinWorld = CartesianState::Random("A", "world");
  ds->set_base_frame(AinWorld);

  // check the setter
  base = ds->get_base_frame();
  EXPECT_STREQ(base.get_name().c_str(), AinWorld.get_name().c_str());
  EXPECT_STREQ(base.get_reference_frame().c_str(), AinWorld.get_reference_frame().c_str());
  EXPECT_NEAR(base.get_pose().norm(), AinWorld.get_pose().norm(), 1e-5);

  // evaluating a current pose B in reference frame A should give zero twist (coincident with attractor)
  CartesianTwist twist = ds->evaluate(BinA);
  EXPECT_STREQ(twist.get_name().c_str(), BinA.get_name().c_str());
  EXPECT_STREQ(twist.get_reference_frame().c_str(), BinA.get_reference_frame().c_str());
  EXPECT_NEAR(twist.data().norm(), 0, 1e-5);

  // evaluating a current pose B in reference frame world should give zero twist (coincident with attractor)
  // PLUS the twist of A in world, expressed as a twist B in world
  auto BinWorld = AinWorld * BinA;
  twist = ds->evaluate(BinWorld);
  EXPECT_STREQ(twist.get_name().c_str(), BinWorld.get_name().c_str());
  EXPECT_STREQ(twist.get_reference_frame().c_str(), BinWorld.get_reference_frame().c_str());
  EXPECT_NEAR(twist.data().norm(), 0 + AinWorld.get_twist().norm(), 1e-5);

  // since the base frame is set to frame A in frame world, evaluating a state with a reference frame that is neither
  // should give an error
  EXPECT_THROW(ds->evaluate(BinWorld.inverse()),
               state_representation::exceptions::IncompatibleReferenceFramesException);
}

TEST_F(PointAttractorTest, StackedMovingReferenceFrames) {
  auto AinWorld = CartesianState::Random("A", "world");
  auto BinA = CartesianState::Random("B", "A");

  auto CinA = CartesianState::Identity("C", "A");
  CinA.set_pose(Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

  ds->set_parameter_value("attractor", BinA);

  // evaluate the twist for a fixed state C in reference frame A
  CartesianTwist twist = ds->evaluate(CinA);

  // the twist should be the same for a moving state C in reference frame A
  // (the DS should always give the velocity from position, not from velocity)
  CinA.set_linear_velocity(Eigen::Vector3d::Random());
  CinA.set_angular_velocity(Eigen::Vector3d::Random());
  CartesianTwist twist2 = ds->evaluate(CinA);

  EXPECT_NEAR(twist.data().norm(), twist2.data().norm(), 1e-5);

  // since reference frame A is moving with respect to the world, and DS gives a twist in reference frame A,
  // the operation to express DS twist in the world frame should be valid.
  ASSERT_NO_THROW(twist = AinWorld * ds->evaluate(CinA));

  // the same operation can be done by setting the base frame of the DS
  ds->set_base_frame(AinWorld);
  auto CinWorld = AinWorld * CinA;
  ASSERT_NO_THROW(twist2 = ds->evaluate(CinWorld));

  // both methods should give the twist in the world frame
  EXPECT_STREQ(twist.get_reference_frame().c_str(), AinWorld.get_reference_frame().c_str());
  EXPECT_STREQ(twist.get_reference_frame().c_str(), twist2.get_reference_frame().c_str());
  EXPECT_NEAR(twist.data().norm(), twist2.data().norm(), 1e-5);

}

TEST_F(PointAttractorTest, UpdateAttractorFrame) {
  CartesianState A, B, C, D;
  A = CartesianState::Random("A", "world");
  B = CartesianState::Random("B", "world");
  C = CartesianState::Random("C", "robot");
  D = CartesianState::Random("D", "robot");

  ds->set_parameter_value("attractor", A);

  // state being evaluated must match the DS base frame, which is by default the attractor reference frame
  EXPECT_NO_THROW(ds->evaluate(B));
  EXPECT_THROW(ds->evaluate(D), state_representation::exceptions::IncompatibleReferenceFramesException);

  // setting the attractor to another point in the same base frame should be fine,
  // but setting it with a different base frame should give an error
  EXPECT_NO_THROW(ds->set_parameter_value("attractor", B));
  EXPECT_THROW(ds->set_parameter_value("attractor", C),
               state_representation::exceptions::IncompatibleReferenceFramesException);

  // after updating the base frame, the attractor reference frame should also be updated
  ds->set_base_frame(CartesianState::Identity(C.get_reference_frame(), C.get_reference_frame()));
  EXPECT_STREQ(ds->get_parameter_value<CartesianPose>("attractor").get_reference_frame().c_str(),
               C.get_reference_frame().c_str());

  // with the new base frame, setting the attractor should succeed / fail accordingly
  EXPECT_THROW(ds->set_parameter_value("attractor", B),
               state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(ds->set_parameter_value("attractor", C));

  // now the evaluation should also succeed when matching the updated base frame
  EXPECT_THROW(ds->evaluate(B), state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(ds->evaluate(D));
}

TEST(JointPointAttractorTest, EmptyConstructor) {
  auto ds = DynamicalSystemFactory<JointState>::create_dynamical_system(
      DynamicalSystemFactory<JointState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
  );
  // construct empty cartesian state DS
  JointState attractor = JointState::Zero("robot", 3);

  // base frame and attractor should be empty
  EXPECT_TRUE(ds->get_parameter_value<JointState>("attractor").is_empty());
  EXPECT_TRUE(ds->get_base_frame().is_empty());
  ds->set_parameter_value("attractor", attractor);
  EXPECT_FALSE(ds->get_parameter_value<JointState>("attractor").is_empty());
  EXPECT_TRUE(ds->get_base_frame().is_empty());
}

TEST(JointPointAttractorTest, EmptyCompatible) {
  auto ds = DynamicalSystemFactory<JointState>::create_dynamical_system(
      DynamicalSystemFactory<JointState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
  );
  JointState state1 = JointState::Zero("robot", 3);
  JointState state2 = JointState("test", 3);
  JointState state3 = JointState("robot", 4);
  JointState state4 = JointState("robot", {"1", "2", "3"});

  // if no attractor is set, an exception is thrown
  EXPECT_THROW(ds->evaluate(state2), dynamical_systems::exceptions::EmptyAttractorException);
  ds->set_parameter_value("attractor", state1);

  EXPECT_THROW(ds->evaluate(state2), state_representation::exceptions::IncompatibleStatesException);
  EXPECT_THROW(ds->evaluate(state3), state_representation::exceptions::IncompatibleStatesException);

  EXPECT_TRUE(ds->is_compatible(state1));
  EXPECT_TRUE(ds->is_compatible(state2));
  EXPECT_FALSE(ds->is_compatible(state3));
  EXPECT_FALSE(ds->is_compatible(state4));
}

TEST(JointPointAttractorTest, Convergence) {
  auto ds = DynamicalSystemFactory<JointState>::create_dynamical_system(
      DynamicalSystemFactory<JointState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
  );
  auto attractor = JointState::Random("robot", 3);
  auto current_state = JointPositions::Random("robot", 3);
  current_state.set_data(10 * current_state.data());

  ds->set_parameter_value("attractor", attractor);
  for (unsigned int i = 0; i < 100; ++i) {
    JointVelocities velocities = ds->evaluate(current_state);
    current_state += 100ms * velocities;
  }
  EXPECT_NEAR(current_state.dist(attractor, JointStateVariable::POSITIONS), 0, 1e-3);
}
