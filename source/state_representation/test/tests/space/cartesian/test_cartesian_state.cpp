#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

using namespace state_representation;

TEST(CartesianStateTest, Constructors) {
  CartesianState empty1;
  EXPECT_EQ(empty1.get_name(), "none");
  EXPECT_EQ(empty1.get_type(), StateType::CARTESIANSTATE);
  EXPECT_TRUE(empty1.is_empty());
  EXPECT_EQ(empty1.get_reference_frame(), "world");
  EXPECT_EQ(empty1.data().norm(), 1);

  CartesianState empty2("test");
  EXPECT_EQ(empty2.get_name(), "test");
  EXPECT_TRUE(empty2.is_empty());
  EXPECT_EQ(empty2.get_reference_frame(), "world");
  EXPECT_EQ(empty2.data().norm(), 1);

  CartesianState empty3("test", "reference");
  EXPECT_EQ(empty3.get_name(), "test");
  EXPECT_TRUE(empty3.is_empty());
  EXPECT_EQ(empty3.get_reference_frame(), "reference");
  EXPECT_EQ(empty3.data().norm(), 1);
}

TEST(CartesianStateTest, IdentityInitialization) {
  CartesianState identity = CartesianState::Identity("test");
  EXPECT_FALSE(identity.is_empty());
  EXPECT_EQ(identity.get_position().norm(), 0);
  EXPECT_EQ(identity.get_orientation().norm(), 1);
  EXPECT_EQ(identity.get_orientation().w(), 1);
  EXPECT_EQ(identity.get_twist().norm(), 0);
  EXPECT_EQ(identity.get_accelerations().norm(), 0);
  EXPECT_EQ(identity.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, RandomStateInitialization) {
  CartesianState random = CartesianState::Random("test");
  EXPECT_NE(random.get_position().norm(), 0);
  EXPECT_EQ(random.get_orientation().norm(), 1);
  EXPECT_NE(random.get_orientation().w(), 0);
  EXPECT_NE(random.get_orientation().x(), 0);
  EXPECT_NE(random.get_orientation().y(), 0);
  EXPECT_NE(random.get_orientation().z(), 0);
  EXPECT_NE(random.get_twist().norm(), 0);
  EXPECT_NE(random.get_accelerations().norm(), 0);
  EXPECT_NE(random.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, CopyConstructor) {
  CartesianState random = CartesianState::Random("test");
  CartesianState copy1(random);
  EXPECT_EQ(random.get_name(), copy1.get_name());
  EXPECT_EQ(random.get_reference_frame(), copy1.get_reference_frame());
  EXPECT_EQ(random.data(), copy1.data());

  CartesianState copy2 = random;
  EXPECT_EQ(random.get_name(), copy2.get_name());
  EXPECT_EQ(random.get_reference_frame(), copy2.get_reference_frame());
  EXPECT_EQ(random.data(), copy2.data());

  CartesianState copy3 = random.copy();
  EXPECT_EQ(random.get_name(), copy3.get_name());
  EXPECT_EQ(random.get_reference_frame(), copy3.get_reference_frame());
  EXPECT_EQ(random.data(), copy3.data());

  CartesianState empty;
  CartesianState copy4(empty);
  EXPECT_TRUE(copy4.is_empty());
  CartesianState copy5 = empty;
  EXPECT_TRUE(copy5.is_empty());
  CartesianState copy6 = empty.copy();
  EXPECT_TRUE(copy6.is_empty());
}

TEST(CartesianStateTest, GetSetFields) {
  CartesianState cs("test");

  // name
  cs.set_name("robot");
  EXPECT_EQ(cs.get_name(), "robot");
  EXPECT_EQ(cs.get_reference_frame(), "world");
  cs.set_reference_frame("base");
  EXPECT_EQ(cs.get_reference_frame(), "base");

  // position
  std::vector<double> position{1, 2, 3};
  cs.set_position(position);
  for (std::size_t i = 0; i < position.size(); ++i) {
    EXPECT_EQ(cs.get_position()(i), position.at(i));
  }
  cs.set_position(1.1, 2.2, 3.3);
  EXPECT_EQ(Eigen::Vector3d(1.1, 2.2, 3.3), cs.get_position());

  // orientation
  Eigen::Vector4d orientation_vec = Eigen::Vector4d::Random().normalized();
  cs.set_orientation(orientation_vec);
  for (auto i = 0; i < orientation_vec.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_orientation_coefficients()(i), orientation_vec(i));
  }
  Eigen::Quaterniond random_orientation = Eigen::Quaterniond::UnitRandom();
  std::vector<double>
      orientation{random_orientation.w(), random_orientation.x(), random_orientation.y(), random_orientation.z()};
  cs.set_orientation(orientation);
  EXPECT_EQ(random_orientation.coeffs(), cs.get_orientation().coeffs());
  EXPECT_THROW(cs.set_orientation(position), exceptions::IncompatibleSizeException);

  auto matrix = cs.get_transformation_matrix();
  Eigen::Vector3d trans = matrix.topRightCorner<3, 1>();
  Eigen::Matrix3d rot = matrix.topLeftCorner<3, 3>();
  Eigen::Vector4d bottom = matrix.bottomLeftCorner<1, 4>();
  EXPECT_EQ(trans, cs.get_position());
  EXPECT_EQ(rot, random_orientation.toRotationMatrix());
  EXPECT_EQ(bottom, Eigen::Vector4d(0, 0, 0, 1));

  // pose
  EXPECT_THROW(cs.set_pose(std::vector<double>(8)), exceptions::IncompatibleSizeException);

  // twist
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Random();
  cs.set_linear_velocity(linear_velocity);
  EXPECT_EQ(cs.get_linear_velocity(), linear_velocity);
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Random();
  cs.set_angular_velocity(angular_velocity);
  EXPECT_EQ(cs.get_angular_velocity(), angular_velocity);
  Eigen::VectorXd twist = Eigen::VectorXd::Random(6);
  cs.set_twist(twist);
  EXPECT_EQ(cs.get_twist(), twist);

  // wrench
  Eigen::Vector3d force = Eigen::Vector3d::Random();
  cs.set_force(force);
  EXPECT_EQ(cs.get_force(), force);
  Eigen::Vector3d torque = Eigen::Vector3d::Random();
  cs.set_torque(torque);
  EXPECT_EQ(cs.get_torque(), torque);
  Eigen::VectorXd wrench = Eigen::VectorXd::Random(6);
  cs.set_wrench(wrench);
  EXPECT_EQ(cs.get_wrench(), wrench);

//  EXPECT_THROW(cs.set_position(Eigen::VectorXd::Zero(4)), exceptions::IncompatibleSizeException);
//  EXPECT_THROW(cs.set_twist(Eigen::VectorXd::Zero(5)), exceptions::IncompatibleSizeException);
//  EXPECT_THROW(cs.set_wrench(Eigen::VectorXd::Zero(7)), exceptions::IncompatibleSizeException);

  cs.set_zero();
  EXPECT_EQ(cs.data().norm(), 1);
  EXPECT_EQ(cs.is_empty(), false);
  cs.set_empty();
  EXPECT_EQ(cs.is_empty(), true);
}

TEST(CartesianStateTest, Compatibility) {
  CartesianState cs1("test");
  CartesianState cs2("robot");
  CartesianState cs3("robot", "test");
  CartesianState cs4("test", "robot");

  EXPECT_FALSE(cs1.is_compatible(cs2));
  EXPECT_FALSE(cs1.is_compatible(cs3));
  EXPECT_FALSE(cs1.is_compatible(cs4));
}

TEST(CartesianStateTest, SetZero) {
  CartesianState random1 = CartesianState::Random("test");
  random1.initialize();
  EXPECT_EQ(random1.data().norm(), 1);

  CartesianState random2 = CartesianState::Random("test");
  random2.set_zero();
  EXPECT_EQ(random2.data().norm(), 1);
}

TEST(CartesianStateTest, GetSetData) {
  CartesianState cs1 = CartesianState::Identity("test");
  CartesianState cs2 = CartesianState::Random("test");
  Eigen::VectorXd concatenated_state(25);
  concatenated_state << cs1.get_pose(), cs1.get_twist(), cs1.get_accelerations(), cs1.get_wrench();
  EXPECT_EQ(concatenated_state, cs1.data());
  for (std::size_t i = 0; i < 25; ++i) {
    EXPECT_FLOAT_EQ(concatenated_state.array()(i), cs1.array()(i));
  }

  cs1.set_data(cs2.data());
  EXPECT_EQ(cs1.data(), cs2.data());

  cs2 = CartesianState::Random("test");
  auto state_vec = cs2.to_std_vector();
  cs1.set_data(state_vec);
  for (std::size_t i = 0; i < state_vec.size(); ++i) {
    EXPECT_FLOAT_EQ(state_vec.at(i), cs1.data()(i));
  }
  EXPECT_THROW(cs1.set_data(Eigen::Vector2d::Zero()), exceptions::IncompatibleSizeException);
}

TEST(CartesianStateTest, ClampVariable) {
  CartesianState state = CartesianState::Identity("test");
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::ORIENTATION), exceptions::NotImplementedException);
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::POSE), exceptions::NotImplementedException);
//  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::ALL), exceptions::NotImplementedException);
  Eigen::Vector3d position(-2.0, 1, 5);
  state.set_position(position);
  state.clamp_state_variable(10.0, CartesianStateVariable::POSITION);
  EXPECT_EQ(state.get_position(), position);
  state.clamp_state_variable(3.0, CartesianStateVariable::POSITION);
  EXPECT_EQ(state.get_position().norm(), 3.0);
  state.clamp_state_variable(10.0, CartesianStateVariable::POSITION, 0.5);
  EXPECT_EQ(state.get_position().norm(), 0.0);
  // TODO
}

TEST(CartesianStateTest, Norms) {
  CartesianState cs = CartesianState::Random("test");
  std::vector<double> norms = cs.norms();
  ASSERT_TRUE(norms.size() == 8);
  EXPECT_FLOAT_EQ(norms[0], cs.get_position().norm());
  EXPECT_FLOAT_EQ(norms[1], cs.get_orientation().norm());
  EXPECT_FLOAT_EQ(norms[2], cs.get_linear_velocity().norm());
  EXPECT_FLOAT_EQ(norms[3], cs.get_angular_velocity().norm());
  EXPECT_FLOAT_EQ(norms[4], cs.get_linear_acceleration().norm());
  EXPECT_FLOAT_EQ(norms[5], cs.get_angular_acceleration().norm());
  EXPECT_FLOAT_EQ(norms[6], cs.get_force().norm());
  EXPECT_FLOAT_EQ(norms[7], cs.get_torque().norm());
}

TEST(CartesianStateTest, Normalize) {
  CartesianState cs = CartesianState::Random("test");
  auto normalized = cs.normalized();
  std::vector<double> norms1 = normalized.norms();
  for (double n: norms1) {
    EXPECT_FLOAT_EQ(n, 1.0);
  }
  cs.normalize();
  std::vector<double> norms2 = cs.norms();
  for (double n: norms2) {
    EXPECT_FLOAT_EQ(n, 1.0);
  }
}

TEST(CartesianStateTest, Distance) {
  CartesianState empty;
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test", "robot");
  EXPECT_THROW(empty.dist(cs1), exceptions::EmptyStateException);
  EXPECT_THROW(cs1.dist(empty), exceptions::EmptyStateException);
  EXPECT_THROW(cs1.dist(cs2), exceptions::IncompatibleReferenceFramesException);

  Eigen::VectorXd data1 = Eigen::VectorXd::Random(25);
  cs1.set_data(data1);
  CartesianState cs3 = CartesianState("test");
  Eigen::VectorXd data3 = Eigen::VectorXd::Random(25);
  cs3.set_data(data3);

  double pos_dist = (data1.head(3) - data3.head(3)).norm();
  double inner_product = cs1.get_orientation().dot(cs3.get_orientation());
  double orient_dist = acos(std::min(1.0, std::max(-1.0, 2 * inner_product * inner_product - 1)));
  double lin_vel_dist = (data1.segment(7, 3) - data3.segment(7, 3)).norm();
  double ang_vel_dist = (data1.segment(10, 3) - data3.segment(10, 3)).norm();
  double lin_acc_dist = (data1.segment(13, 3) - data3.segment(13, 3)).norm();
  double ang_acc_dist = (data1.segment(16, 3) - data3.segment(16, 3)).norm();
  double force_dist = (data1.segment(19, 3) - data3.segment(19, 3)).norm();
  double torque_dist = (data1.segment(22, 3) - data3.segment(22, 3)).norm();
  double total_dist =
      pos_dist + orient_dist + lin_vel_dist + ang_vel_dist + lin_acc_dist + ang_acc_dist + force_dist + torque_dist;

  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::POSITION), pos_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ORIENTATION), orient_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::POSE), pos_dist + orient_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::TWIST), lin_vel_dist + ang_vel_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ACCELERATIONS), lin_acc_dist + ang_acc_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::WRENCH), force_dist + torque_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ALL), total_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3), cs3.dist(cs1));
}

TEST(CartesianStateTest, Inverse) {
  // TODO
}

TEST(CartesianStateTest, Addition) {
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test");
  CartesianState cs3 = CartesianState::Random("test", "reference");
  EXPECT_THROW(cs1 + cs3, exceptions::IncompatibleReferenceFramesException);

  CartesianState csum = cs1 + cs2;
  EXPECT_EQ(csum.get_position(), cs1.get_position() + cs2.get_position());
  Eigen::Quaterniond orientation = (cs1.get_orientation().dot(cs2.get_orientation()) > 0) ? cs2.get_orientation()
                                                                                          : Eigen::Quaterniond(-cs2.get_orientation().coeffs());
  orientation = cs1.get_orientation() * orientation;
  EXPECT_TRUE(csum.get_orientation().coeffs().isApprox(orientation.coeffs()));
  EXPECT_EQ(csum.get_twist(), cs1.get_twist() + cs2.get_twist());
  EXPECT_EQ(csum.get_accelerations(), cs1.get_accelerations() + cs2.get_accelerations());
  EXPECT_EQ(csum.get_wrench(), cs1.get_wrench() + cs2.get_wrench());

  cs1 += cs2;
  EXPECT_EQ(csum.get_position(), cs1.get_position());
  EXPECT_TRUE(csum.get_orientation().coeffs().isApprox(cs1.get_orientation().coeffs()));
  EXPECT_EQ(csum.get_twist(), cs1.get_twist());
  EXPECT_EQ(csum.get_accelerations(), cs1.get_accelerations());
  EXPECT_EQ(csum.get_wrench(), cs1.get_wrench());
}

TEST(CartesianStateTest, Subtraction) {
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test");
  CartesianState cs3 = CartesianState::Random("test", "reference");
  EXPECT_THROW(cs1 - cs3, exceptions::IncompatibleReferenceFramesException);

  CartesianState cdiff = cs1 - cs2;
  EXPECT_EQ(cdiff.get_position(), cs1.get_position() - cs2.get_position());
  Eigen::Quaterniond orientation = (cs1.get_orientation().dot(cs2.get_orientation()) > 0) ? cs2.get_orientation()
                                                                                          : Eigen::Quaterniond(-cs2.get_orientation().coeffs());
  orientation = cs1.get_orientation() * orientation.conjugate();
  EXPECT_TRUE(cdiff.get_orientation().coeffs().isApprox(orientation.coeffs()));
  EXPECT_EQ(cdiff.get_twist(), cs1.get_twist() - cs2.get_twist());
  EXPECT_EQ(cdiff.get_accelerations(), cs1.get_accelerations() - cs2.get_accelerations());
  EXPECT_EQ(cdiff.get_wrench(), cs1.get_wrench() - cs2.get_wrench());

  cs1 -= cs2;
  EXPECT_EQ(cdiff.get_position(), cs1.get_position());
  EXPECT_TRUE(cdiff.get_orientation().coeffs().isApprox(cs1.get_orientation().coeffs()));
  EXPECT_EQ(cdiff.get_twist(), cs1.get_twist());
  EXPECT_EQ(cdiff.get_accelerations(), cs1.get_accelerations());
  EXPECT_EQ(cdiff.get_wrench(), cs1.get_wrench());
}

TEST(CartesianStateTest, ScalarMultiplication) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = scalar * cs;
  EXPECT_EQ(cscaled.get_position(), scalar * cs.get_position());
  Eigen::Quaterniond qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), scalar / 2.);
  EXPECT_TRUE(cscaled.get_orientation().coeffs().isApprox(qscaled.coeffs()));
  EXPECT_EQ(cscaled.get_twist(), scalar * cs.get_twist());
  EXPECT_EQ(cscaled.get_accelerations(), scalar * cs.get_accelerations());
  EXPECT_EQ(cscaled.get_wrench(), scalar * cs.get_wrench());
  EXPECT_EQ((cs * scalar).data(), cscaled.data());
  cs *= scalar;
  EXPECT_EQ(cscaled.data(), cs.data());

  CartesianState empty;
  EXPECT_THROW(scalar * empty, exceptions::EmptyStateException);
}


TEST(CartesianStateTest, ScalarDivision) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = cs / scalar;
  EXPECT_EQ(cscaled.get_position(), cs.get_position() / scalar);
  Eigen::Quaterniond qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), 1.0 / (2. * scalar));
  EXPECT_TRUE(cscaled.get_orientation().coeffs().isApprox(qscaled.coeffs()));
  EXPECT_EQ(cscaled.get_twist(), cs.get_twist() / scalar);
  EXPECT_EQ(cscaled.get_accelerations(), cs.get_accelerations() / scalar);
  EXPECT_EQ(cscaled.get_wrench(), cs.get_wrench() / scalar);
  cs /= scalar;
  EXPECT_EQ(cscaled.data(), cs.data());

  EXPECT_THROW(cs / 0.0, std::runtime_error);

  CartesianState empty;
  EXPECT_THROW(empty / scalar, exceptions::EmptyStateException);
}
