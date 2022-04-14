#include <gtest/gtest.h>

#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "dynamical_systems/exceptions/EmptyBaseFrameException.hpp"
#include "dynamical_systems/exceptions/EmptyAttractorException.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/parameters/Parameter.hpp"

using namespace dynamical_systems;
using namespace state_representation;
using namespace std::literals::chrono_literals;

class CircularDSTest : public testing::Test {
protected:
  void SetUp() override {
    ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::CIRCULAR);
    current_pose = CartesianPose("A", 10 * Eigen::Vector3d::Random());
    center = CartesianPose::Identity("B");
    limit_cycle.set_center_pose(center);
    limit_cycle.set_axis_lengths({radius, radius});
  }

  std::shared_ptr<IDynamicalSystem<CartesianState>> ds;
  Ellipsoid limit_cycle = Ellipsoid("limit_cycle");
  CartesianPose current_pose;
  CartesianPose center;
  double radius = 10;
  unsigned int nb_steps = 2000;
  std::chrono::milliseconds dt = 10ms;
  double linear_tol = 1e-3;
};

TEST_F(CircularDSTest, TestPositionOnRadius) {
  ds->set_parameter_value("limit_cycle", limit_cycle);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    CartesianTwist twist = ds->evaluate(current_pose);
    twist.clamp(10, 10, 0.001, 0.001);
    current_pose += dt * twist;
  }

  EXPECT_NEAR(current_pose.dist(center, CartesianStateVariable::POSITION), radius, linear_tol);
}

TEST_F(CircularDSTest, EmptyConstructor) {
  // base frame and attractor should be empty
  EXPECT_TRUE(ds->get_parameter_value<Ellipsoid>("limit_cycle").get_center_pose().is_empty());
  EXPECT_TRUE(ds->get_base_frame().is_empty());

  ds->set_parameter_value("limit_cycle", limit_cycle);
  EXPECT_FALSE(ds->get_parameter_value<Ellipsoid>("limit_cycle").get_center_pose().is_empty());
  EXPECT_FALSE(ds->get_base_frame().is_empty());
  // when attractor was set without a base frame, expect base frame to be identity with name / reference_frame of attractor
  EXPECT_EQ(ds->get_base_frame().get_name(), center.get_reference_frame());
  EXPECT_EQ(ds->get_base_frame().get_reference_frame(), center.get_reference_frame());
  EXPECT_EQ(ds->get_base_frame().get_transformation_matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(CircularDSTest, EvaluateCompatibility) {
  CartesianState state1 = CartesianState::Identity("world", "A");
  CartesianState state2("D", "C");
  CartesianState state3("C", "A");
  CartesianState state4("C", "world");
  // if no base frame is set, an exception is thrown
  EXPECT_THROW(ds->evaluate(state1), dynamical_systems::exceptions::EmptyBaseFrameException);
  ds->set_base_frame(state1);
  // if cartesian state is incompatible, an exception is thrown
  EXPECT_THROW(ds->evaluate(state2), state_representation::exceptions::IncompatibleReferenceFramesException);
  // if cartesian state needs to be transformed to other frame first and is empty, an exception is thrown
  EXPECT_THROW(ds->evaluate(state3), state_representation::exceptions::EmptyStateException);
  // if no attractor is set, an exception is thrown
  EXPECT_THROW(ds->evaluate(state4), dynamical_systems::exceptions::EmptyAttractorException);

  ds->set_parameter_value("limit_cycle", limit_cycle);
  EXPECT_TRUE(ds->is_compatible(state1));
  EXPECT_FALSE(ds->is_compatible(state2));
  EXPECT_TRUE(ds->is_compatible(state3));
  EXPECT_TRUE(ds->is_compatible(state4));
}

TEST_F(CircularDSTest, TestPositionOnRadiusRandomCenter) {
  limit_cycle.set_center_position(Eigen::Vector3d::Random());
  limit_cycle.set_center_orientation(Eigen::Quaterniond::UnitRandom());
  ds->set_parameter_value("limit_cycle", limit_cycle);

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = ds->evaluate(current_pose);
    twist.clamp(10, 10, 0.001, 0.001);
    current_pose += dt * twist;
  }

  EXPECT_NEAR(current_pose.dist(limit_cycle.get_center_pose(), CartesianStateVariable::POSITION), radius, linear_tol);
}

TEST_F(CircularDSTest, SetCenterAndBase) {
  Ellipsoid cycle("B", "A");
  auto BinA = CartesianState::Identity("B", "A");
  auto CinA = CartesianState::Identity("C", "A");
  auto CinB = CartesianState::Identity("C", "B");

  ds->set_parameter_value("limit_cycle", cycle);
  EXPECT_STREQ(ds->get_parameter_value<Ellipsoid>("limit_cycle").get_center_pose().get_name().c_str(),
               BinA.get_name().c_str());
  EXPECT_STREQ(ds->get_parameter_value<Ellipsoid>("limit_cycle").get_center_pose().get_reference_frame().c_str(),
               BinA.get_reference_frame().c_str());

  // evaluating a state is only valid if it matches the base reference frame
  EXPECT_NO_THROW(ds->evaluate(CinA));
  EXPECT_THROW(ds->evaluate(CinB), state_representation::exceptions::IncompatibleReferenceFramesException);

  // setting the center is only valid if it matches the base reference frame
  cycle.set_center_state(CinA);
  EXPECT_NO_THROW(ds->set_parameter_value("limit_cycle", cycle));
  Ellipsoid cycle2("C", "B");
  EXPECT_THROW(ds->set_parameter_value("limit_cycle", cycle2),
               state_representation::exceptions::IncompatibleReferenceFramesException);

  // setting the base frame should also update the reference frame of the center
  ASSERT_NO_THROW(ds->set_base_frame(CinB));
  EXPECT_STREQ(ds->get_base_frame().get_name().c_str(), CinB.get_name().c_str());
  EXPECT_STREQ(ds->get_base_frame().get_reference_frame().c_str(), CinB.get_reference_frame().c_str());
  EXPECT_STREQ(ds->get_parameter_value<Ellipsoid>("limit_cycle").get_center_pose().get_reference_frame().c_str(),
               ds->get_base_frame().get_name().c_str());

  // now the base frame is C, setting a center should only work if the reference frame of the center is also C
  EXPECT_THROW(ds->set_parameter_value("limit_cycle", cycle),
               state_representation::exceptions::IncompatibleReferenceFramesException);
  EXPECT_NO_THROW(ds->set_parameter_value("limit_cycle", cycle2));
  EXPECT_NO_THROW(ds->set_parameter_value("limit_cycle", Ellipsoid("B", "C")));
}
