#include <functional>
#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

using namespace state_representation;

static void assert_name_empty_frame_equal(
    const CartesianState& state1, const std::string& name, bool empty, const std::string& reference_frame
) {
  EXPECT_EQ(state1.get_name(), name);
  EXPECT_EQ(state1.get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(state1.is_empty(), empty);
  EXPECT_EQ(state1.get_reference_frame(), reference_frame);
}

static void assert_name_frame_data_equal(const CartesianState& state1, const CartesianState& state2) {
  EXPECT_EQ(state1.get_name(), state2.get_name());
  EXPECT_EQ(state1.get_reference_frame(), state2.get_reference_frame());
  EXPECT_TRUE(state1.data().isApprox(state2.data()));
}

template<int dim>
void test_clamping(
    CartesianState& state, std::function<Eigen::Matrix<double, dim, 1>(const CartesianState&)> getter,
    std::function<void(CartesianState&, Eigen::Matrix<double, dim, 1>)> setter, const CartesianStateVariable& type
) {
  Eigen::Matrix<double, dim, 1> data;
  if (dim == 3) {
    data << -2.0, 1, 5;
  } else if (dim == 6) {
    data << -2.0, 1, 5, 1, -3.0, 2.4;
  }
  setter(state, data);
  state.clamp_state_variable(10.0, type);
  EXPECT_TRUE(getter(state).isApprox(data));
  state.clamp_state_variable(3.0, type);
  EXPECT_FLOAT_EQ(getter(state).norm(), 3.0);
  state.clamp_state_variable(10.0, type, 0.5);
  EXPECT_FLOAT_EQ(getter(state).norm(), 0.0);
}

TEST(CartesianStateTest, Constructors) {
  CartesianState empty1;
  EXPECT_EQ(empty1.get_type(), StateType::CARTESIAN_STATE);
  assert_name_empty_frame_equal(empty1, "", true, "world");
  EXPECT_FLOAT_EQ(empty1.data().norm(), 1);

  CartesianState empty2("test");
  assert_name_empty_frame_equal(empty2, "test", true, "world");
  EXPECT_FLOAT_EQ(empty2.data().norm(), 1);

  CartesianState empty3("test", "reference");
  assert_name_empty_frame_equal(empty3, "test", true, "reference");
  EXPECT_FLOAT_EQ(empty3.data().norm(), 1);
}

TEST(CartesianStateTest, IdentityInitialization) {
  CartesianState identity = CartesianState::Identity("test");
  EXPECT_FALSE(identity.is_empty());
  EXPECT_FLOAT_EQ(identity.get_position().norm(), 0);
  EXPECT_FLOAT_EQ(identity.get_orientation().norm(), 1);
  EXPECT_FLOAT_EQ(identity.get_orientation().w(), 1);
  EXPECT_FLOAT_EQ(identity.get_twist().norm(), 0);
  EXPECT_FLOAT_EQ(identity.get_acceleration().norm(), 0);
  EXPECT_FLOAT_EQ(identity.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, RandomStateInitialization) {
  CartesianState random = CartesianState::Random("test");
  EXPECT_NE(random.get_position().norm(), 0);
  EXPECT_FLOAT_EQ(random.get_orientation().norm(), 1);
  EXPECT_NE(random.get_orientation().w(), 0);
  EXPECT_NE(random.get_orientation().x(), 0);
  EXPECT_NE(random.get_orientation().y(), 0);
  EXPECT_NE(random.get_orientation().z(), 0);
  EXPECT_NE(random.get_twist().norm(), 0);
  EXPECT_NE(random.get_acceleration().norm(), 0);
  EXPECT_NE(random.get_wrench().norm(), 0);
}

TEST(CartesianStateTest, CopyConstructor) {
  CartesianState random = CartesianState::Random("test");
  CartesianState copy1(random);
  assert_name_frame_data_equal(random, copy1);

  CartesianState copy2 = random;
  assert_name_frame_data_equal(random, copy2);

  CartesianState copy3 = random.copy();
  assert_name_frame_data_equal(random, copy3);

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
  static Eigen::Vector3d data;
  static std::vector<double> std_data(3);

  // name
  cs.set_name("robot");
  EXPECT_EQ(cs.get_name(), "robot");
  EXPECT_EQ(cs.get_reference_frame(), "world");
  cs.set_reference_frame("base");
  EXPECT_EQ(cs.get_reference_frame(), "base");

  // position
  std_data = {1, 2, 3};
  cs.set_position(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_position()(i), std_data.at(i));
  }
  cs.set_position(1.1, 2.2, 3.3);
  EXPECT_TRUE(Eigen::Vector3d(1.1, 2.2, 3.3).isApprox(cs.get_position()));
  EXPECT_THROW(cs.set_position(std::vector<double>{1, 2, 3, 4}), exceptions::IncompatibleSizeException);

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
  EXPECT_TRUE(random_orientation.coeffs().isApprox(cs.get_orientation().coeffs()));
  random_orientation = Eigen::Quaterniond::UnitRandom();
  cs.set_orientation(random_orientation.w(), random_orientation.x(), random_orientation.y(), random_orientation.z());
  EXPECT_TRUE(random_orientation.coeffs().isApprox(cs.get_orientation().coeffs()));
  EXPECT_THROW(cs.set_orientation(std_data), exceptions::IncompatibleSizeException);

  auto matrix = cs.get_transformation_matrix();
  Eigen::Vector3d trans = matrix.topRightCorner<3, 1>();
  Eigen::Matrix3d rot = matrix.topLeftCorner<3, 3>();
  Eigen::Vector4d bottom = matrix.bottomLeftCorner<1, 4>();
  EXPECT_TRUE(trans.isApprox(cs.get_position()));
  EXPECT_TRUE(rot.isApprox(random_orientation.toRotationMatrix()));
  EXPECT_TRUE(bottom.isApprox(Eigen::Vector4d(0, 0, 0, 1)));

  // pose
  EXPECT_THROW(cs.set_pose(std::vector<double>(8)), exceptions::IncompatibleSizeException);

  // linear velocity
  data = Eigen::Vector3d::Random();
  cs.set_linear_velocity(data);
  EXPECT_TRUE(cs.get_linear_velocity().isApprox(data));
  std_data = {2, 3, 4};
  cs.set_linear_velocity(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_linear_velocity()(i), std_data.at(i));
  }
  cs.set_linear_velocity(2.1, 3.2, 4.3);
  EXPECT_TRUE(Eigen::Vector3d(2.1, 3.2, 4.3).isApprox(cs.get_linear_velocity()));
  EXPECT_THROW(cs.set_linear_velocity(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // angular velocity
  data = Eigen::Vector3d::Random();
  cs.set_angular_velocity(data);
  EXPECT_TRUE(cs.get_angular_velocity().isApprox(data));
  std_data = {3, 4, 5};
  cs.set_angular_velocity(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_angular_velocity()(i), std_data.at(i));
  }
  cs.set_angular_velocity(3.1, 4.2, 5.3);
  EXPECT_TRUE(Eigen::Vector3d(3.1, 4.2, 5.3).isApprox(cs.get_angular_velocity()));
  EXPECT_THROW(cs.set_angular_velocity(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // twist
  Eigen::VectorXd twist_eigen = Eigen::VectorXd::Random(6);
  cs.set_twist(twist_eigen);
  EXPECT_TRUE(cs.get_twist().isApprox(twist_eigen));
  std::vector<double> twist{4, 5, 6, 7, 8, 9};
  cs.set_twist(twist);
  for (std::size_t i = 0; i < twist.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_twist()(i), twist.at(i));
  }
  EXPECT_THROW(cs.set_twist(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // linear acceleration
  data = Eigen::Vector3d::Random();
  cs.set_linear_acceleration(data);
  EXPECT_TRUE(cs.get_linear_acceleration().isApprox(data));
  std_data = {5, 6, 7};
  cs.set_linear_acceleration(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_linear_acceleration()(i), std_data.at(i));
  }
  cs.set_linear_acceleration(5.1, 6.2, 7.3);
  EXPECT_TRUE(Eigen::Vector3d(5.1, 6.2, 7.3).isApprox(cs.get_linear_acceleration()));
  EXPECT_THROW(cs.set_linear_acceleration(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // angular acceleration
  data = Eigen::Vector3d::Random();
  cs.set_angular_acceleration(data);
  EXPECT_TRUE(cs.get_angular_acceleration().isApprox(data));
  std_data = {6, 7, 8};
  cs.set_angular_acceleration(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_angular_acceleration()(i), std_data.at(i));
  }
  cs.set_angular_acceleration(6.1, 7.2, 8.3);
  EXPECT_TRUE(Eigen::Vector3d(6.1, 7.2, 8.3).isApprox(cs.get_angular_acceleration()));
  EXPECT_THROW(cs.set_angular_acceleration(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // acceleration
  Eigen::VectorXd acceleration_eigen = Eigen::VectorXd::Random(6);
  cs.set_acceleration(acceleration_eigen);
  EXPECT_TRUE(cs.get_acceleration().isApprox(acceleration_eigen));
  std::vector<double> acceleration{7, 8, 9, 10, 11, 12};
  cs.set_acceleration(acceleration);
  for (std::size_t i = 0; i < acceleration.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_acceleration()(i), acceleration.at(i));
  }
  EXPECT_THROW(cs.set_acceleration(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // force
  data = Eigen::Vector3d::Random();
  cs.set_force(data);
  EXPECT_TRUE(cs.get_force().isApprox(data));
  std_data = {8, 9, 10};
  cs.set_force(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_force()(i), std_data.at(i));
  }
  cs.set_force(8.1, 9.2, 10.3);
  EXPECT_TRUE(Eigen::Vector3d(8.1, 9.2, 10.3).isApprox(cs.get_force()));
  EXPECT_THROW(cs.set_force(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // torque
  data = Eigen::Vector3d::Random();
  cs.set_torque(data);
  EXPECT_TRUE(cs.get_torque().isApprox(data));
  std_data = {9, 10, 11};
  cs.set_torque(std_data);
  for (std::size_t i = 0; i < std_data.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_torque()(i), std_data.at(i));
  }
  cs.set_torque(9.1, 10.2, 11.3);
  EXPECT_TRUE(Eigen::Vector3d(9.1, 10.2, 11.3).isApprox(cs.get_torque()));
  EXPECT_THROW(cs.set_torque(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  // wrench
  Eigen::VectorXd wrench_eigen = Eigen::VectorXd::Random(6);
  cs.set_wrench(wrench_eigen);
  EXPECT_TRUE(cs.get_wrench().isApprox(wrench_eigen));
  std::vector<double> wrench{10, 11, 12, 13, 14, 15};
  cs.set_wrench(wrench);
  for (std::size_t i = 0; i < wrench.size(); ++i) {
    EXPECT_FLOAT_EQ(cs.get_wrench()(i), wrench.at(i));
  }
  EXPECT_THROW(cs.set_wrench(std::vector<double>(4)), exceptions::IncompatibleSizeException);

  cs.set_zero();
  EXPECT_FLOAT_EQ(cs.data().norm(), 1);
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
  EXPECT_FLOAT_EQ(random1.data().norm(), 1);

  CartesianState random2 = CartesianState::Random("test");
  random2.set_zero();
  EXPECT_FLOAT_EQ(random2.data().norm(), 1);
}

TEST(CartesianStateTest, GetSetData) {
  CartesianState cs1 = CartesianState::Identity("test");
  CartesianState cs2 = CartesianState::Random("test");
  Eigen::VectorXd concatenated_state(25);
  concatenated_state << cs1.get_pose(), cs1.get_twist(), cs1.get_acceleration(), cs1.get_wrench();
  EXPECT_TRUE(concatenated_state.isApprox(cs1.data()));
  for (std::size_t i = 0; i < 25; ++i) {
    EXPECT_FLOAT_EQ(concatenated_state.array()(i), cs1.array()(i));
  }

  cs1.set_data(cs2.data());
  EXPECT_TRUE(cs1.data().isApprox(cs2.data()));

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
  EXPECT_THROW(state.clamp_state_variable(1, CartesianStateVariable::ALL), exceptions::NotImplementedException);

  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_position(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_position(data); },
      CartesianStateVariable::POSITION
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_linear_velocity(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_linear_velocity(data); },
      CartesianStateVariable::LINEAR_VELOCITY
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_angular_velocity(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_angular_velocity(data); },
      CartesianStateVariable::ANGULAR_VELOCITY
  );
  test_clamping<6>(
      state, [](const CartesianState& state) -> Eigen::Matrix<double, 6, 1> { return state.get_twist(); },
      [](CartesianState& state, const Eigen::Matrix<double, 6, 1>& data) { state.set_twist(data); },
      CartesianStateVariable::TWIST
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_linear_acceleration(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_linear_acceleration(data); },
      CartesianStateVariable::LINEAR_ACCELERATION
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_angular_acceleration(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_angular_acceleration(data); },
      CartesianStateVariable::ANGULAR_ACCELERATION
  );
  test_clamping<6>(
      state, [](const CartesianState& state) -> Eigen::MatrixXd { return state.get_acceleration(); },
      [](CartesianState& state, const Eigen::MatrixXd& data) { state.set_acceleration(data); },
      CartesianStateVariable::ACCELERATION
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_force(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_force(data); }, CartesianStateVariable::FORCE
  );
  test_clamping<3>(
      state, [](const CartesianState& state) -> const Eigen::Vector3d& { return state.get_torque(); },
      [](CartesianState& state, const Eigen::Vector3d& data) { state.set_torque(data); }, CartesianStateVariable::TORQUE
  );
  test_clamping<6>(
      state, [](const CartesianState& state) -> Eigen::MatrixXd { return state.get_wrench(); },
      [](CartesianState& state, const Eigen::MatrixXd& data) { state.set_wrench(data); }, CartesianStateVariable::WRENCH
  );
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
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ACCELERATION), lin_acc_dist + ang_acc_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::WRENCH), force_dist + torque_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3, CartesianStateVariable::ALL), total_dist);
  EXPECT_FLOAT_EQ(cs1.dist(cs3), cs3.dist(cs1));
}

TEST(CartesianStateTest, TestInverseStates) {
CartesianState a_state_b = CartesianState::Random("B", "A");
auto b_state_a = a_state_b.inverse();
auto expect_null = a_state_b * b_state_a;

EXPECT_NEAR(expect_null.get_pose().norm(), 1, 1e-5);
EXPECT_NEAR(expect_null.get_linear_velocity().norm(), 0, 1e-5);
EXPECT_NEAR(expect_null.get_angular_velocity().norm(), 0, 1e-5);
EXPECT_NEAR(expect_null.get_linear_acceleration().norm(), 0, 1e-5);
EXPECT_NEAR(expect_null.get_angular_acceleration().norm(), 0, 1e-5);
EXPECT_NEAR(expect_null.get_force().norm(), 0, 1e-5);
EXPECT_NEAR(expect_null.get_torque().norm(), 0, 1e-5);

// conservation of power must hold
auto power = a_state_b.get_twist().transpose() * a_state_b.get_wrench();
auto power_inverse = b_state_a.get_twist().transpose() * b_state_a.get_wrench();
EXPECT_NEAR(power, power_inverse, 1e-5);
}

TEST(CartesianStateTest, Addition) {
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test");
  CartesianState cs3 = CartesianState::Random("test", "reference");
  EXPECT_THROW(cs1 + cs3, exceptions::IncompatibleReferenceFramesException);

  CartesianState csum = cs1 + cs2;
  EXPECT_TRUE(csum.get_position().isApprox(cs1.get_position() + cs2.get_position()));
  Eigen::Quaterniond
      orientation = (cs1.get_orientation().dot(cs2.get_orientation()) > 0) ? cs2.get_orientation() : Eigen::Quaterniond(
      -cs2.get_orientation().coeffs());
  orientation = cs1.get_orientation() * orientation;
  EXPECT_TRUE(csum.get_orientation().coeffs().isApprox(orientation.coeffs()));
  EXPECT_TRUE(csum.get_twist().isApprox(cs1.get_twist() + cs2.get_twist()));
  EXPECT_TRUE(csum.get_acceleration().isApprox(cs1.get_acceleration() + cs2.get_acceleration()));
  EXPECT_TRUE(csum.get_wrench().isApprox(cs1.get_wrench() + cs2.get_wrench()));

  cs1 += cs2;
  EXPECT_TRUE(cs1.data().isApprox(csum.data()));
}

TEST(CartesianStateTest, Subtraction) {
  CartesianState cs1 = CartesianState::Random("test");
  CartesianState cs2 = CartesianState::Random("test");
  CartesianState cs3 = CartesianState::Random("test", "reference");
  EXPECT_THROW(cs1 - cs3, exceptions::IncompatibleReferenceFramesException);

  CartesianState cdiff = cs1 - cs2;
  EXPECT_TRUE(cdiff.get_position().isApprox(cs1.get_position() - cs2.get_position()));
  Eigen::Quaterniond
      orientation = (cs1.get_orientation().dot(cs2.get_orientation()) > 0) ? cs2.get_orientation() : Eigen::Quaterniond(
      -cs2.get_orientation().coeffs());
  orientation = cs1.get_orientation() * orientation.conjugate();
  EXPECT_TRUE(cdiff.get_orientation().coeffs().isApprox(orientation.coeffs()));
  EXPECT_TRUE(cdiff.get_twist().isApprox(cs1.get_twist() - cs2.get_twist()));
  EXPECT_TRUE(cdiff.get_acceleration().isApprox(cs1.get_acceleration() - cs2.get_acceleration()));
  EXPECT_TRUE(cdiff.get_wrench().isApprox(cs1.get_wrench() - cs2.get_wrench()));

  cs1 -= cs2;
  EXPECT_TRUE(cs1.data().isApprox(cdiff.data()));
}

TEST(CartesianStateTest, Multiplication) {
  CartesianState a_state_b = CartesianState::Identity("B", "A");
  CartesianState b_state_c = CartesianState::Random("C", "B");

  auto a_state_c = a_state_b * b_state_c;

  // if the base frame is identity, the transformation should not affect the data
  EXPECT_TRUE(a_state_c.get_pose().isApprox(b_state_c.get_pose()));
  EXPECT_TRUE(a_state_c.get_accelerations().isApprox(b_state_c.get_accelerations()));
  EXPECT_TRUE(a_state_c.get_twist().isApprox(b_state_c.get_twist()));
  EXPECT_TRUE(a_state_c.get_wrench().isApprox(b_state_c.get_wrench()));

  a_state_b = CartesianState::Random("B", "A");
  a_state_c = a_state_b * b_state_c;

  // conservation of power dictates that the product of force and velocity of
  // frame C must be the same in any reference frame.
  double b_power_c = b_state_c.get_twist().transpose() * b_state_c.get_wrench();
  double projected_b_power_c = (a_state_c.get_twist() - a_state_b.get_twist()).transpose()
      * (a_state_c.get_wrench() - a_state_b.get_wrench());

  EXPECT_NEAR(b_power_c, projected_b_power_c, 1e-3);
}

TEST(CartesianStateTest, ScalarMultiplication) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = scalar * cs;
  EXPECT_TRUE(cscaled.get_position().isApprox(scalar * cs.get_position()));
  Eigen::Quaterniond qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), scalar / 2.);
  EXPECT_TRUE(cscaled.get_orientation().coeffs().isApprox(qscaled.coeffs()));
  EXPECT_TRUE(cscaled.get_twist().isApprox(scalar * cs.get_twist()));
  EXPECT_TRUE(cscaled.get_acceleration().isApprox(scalar * cs.get_acceleration()));
  EXPECT_TRUE(cscaled.get_wrench().isApprox(scalar * cs.get_wrench()));
  EXPECT_TRUE((cs * scalar).data().isApprox(cscaled.data()));
  cs *= scalar;
  EXPECT_TRUE(cscaled.data().isApprox(cs.data()));

  CartesianState empty;
  EXPECT_THROW(scalar * empty, exceptions::EmptyStateException);
}

TEST(CartesianStateTest, ScalarDivision) {
  double scalar = 2;
  CartesianState cs = CartesianState::Random("test");
  CartesianState cscaled = cs / scalar;
  EXPECT_TRUE(cscaled.get_position().isApprox(cs.get_position() / scalar));
  Eigen::Quaterniond qscaled = math_tools::exp(math_tools::log(cs.get_orientation()), 1.0 / (2. * scalar));
  EXPECT_TRUE(cscaled.get_orientation().coeffs().isApprox(qscaled.coeffs()));
  EXPECT_TRUE(cscaled.get_twist().isApprox(cs.get_twist() / scalar));
  EXPECT_TRUE(cscaled.get_acceleration().isApprox(cs.get_acceleration() / scalar));
  EXPECT_TRUE(cscaled.get_wrench().isApprox(cs.get_wrench() / scalar));
  cs /= scalar;
  EXPECT_TRUE(cscaled.data().isApprox(cs.data()));

  EXPECT_THROW(cs / 0.0, std::runtime_error);

  CartesianState empty;
  EXPECT_THROW(empty / scalar, exceptions::EmptyStateException);
}