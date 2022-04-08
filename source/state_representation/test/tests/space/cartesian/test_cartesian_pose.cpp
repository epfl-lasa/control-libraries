#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianPose.hpp"

using namespace state_representation;

static void expect_only_pose(CartesianPose& pose) {
  EXPECT_EQ(static_cast<CartesianState&>(pose).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose).get_acceleration().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose).get_wrench().norm(), 0);
}

static void expect_near(const Eigen::VectorXd& eigen1, const Eigen::VectorXd& eigen2, double tol = 1e-5) {
  ASSERT_TRUE(eigen1.size() == eigen2.size());
  for (int i = 0; i < eigen1.size(); ++i) {
    EXPECT_NEAR(eigen1(i), eigen2(i), tol);
  }
}

class CartesianPoseTestClass : public testing::Test {
protected:
  void SetUp() override {
    Eigen::Vector3d pos1(1, 2, 3);
    Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
    tf1 = CartesianPose("t1", pos1, rot1);
    Eigen::Vector3d pos2(4, 5, 6);
    Eigen::Quaterniond rot2 = Eigen::Quaterniond::Identity();
    tf2 = CartesianPose("t2", pos2, rot2, "t1");
  }

  CartesianPose tf1;
  CartesianPose tf2;

  double tol = 1e-5;
};

TEST(CartesianPoseTest, RandomPoseInitialization) {
  CartesianPose random = CartesianPose::Random("test");
  EXPECT_EQ(random.get_type(), StateType::CARTESIAN_POSE);
  EXPECT_NE(random.get_position().norm(), 0);
  EXPECT_NE(random.get_orientation().w(), 0);
  EXPECT_NE(random.get_orientation().x(), 0);
  EXPECT_NE(random.get_orientation().y(), 0);
  EXPECT_NE(random.get_orientation().z(), 0);
  expect_only_pose(random);
}

TEST(CartesianPoseTest, CopyPose) {
  CartesianPose pose1 = CartesianPose::Random("test");
  CartesianPose pose2(pose1);
  EXPECT_EQ(pose1.get_type(), pose2.get_type());
  EXPECT_EQ(pose1.get_name(), pose2.get_name());
  EXPECT_EQ(pose1.get_reference_frame(), pose2.get_reference_frame());
  EXPECT_EQ(pose1.data(), pose2.data());
  expect_only_pose(pose2);

  CartesianPose pose3 = pose1;
  EXPECT_EQ(pose1.get_type(), pose3.get_type());
  EXPECT_EQ(pose1.get_name(), pose3.get_name());
  EXPECT_EQ(pose1.get_reference_frame(), pose3.get_reference_frame());
  EXPECT_EQ(pose1.data(), pose3.data());
  expect_only_pose(pose3);

  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<CartesianState&>(pose1).set_twist(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(pose1).set_acceleration(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(pose1).set_wrench(Eigen::VectorXd::Random(6));
  CartesianPose pose4 = pose1;
  EXPECT_EQ(pose1.data(), pose4.data());
  expect_only_pose(pose4);

  // copy a state, only the pose variables should be non 0
  CartesianPose pose5 = CartesianState::Random("test");
  expect_only_pose(pose5);

  CartesianPose pose6;
  EXPECT_TRUE(pose6.is_empty());
  CartesianPose pose7 = pose6;
  EXPECT_TRUE(pose7.is_empty());
}

TEST(CartesianPoseTest, SetData) {
  CartesianPose cp1 = CartesianPose::Identity("test");
  CartesianPose cp2 = CartesianPose::Random("test");
  cp1.set_data(cp2.data());
  EXPECT_EQ(cp2.data(), cp1.data());

  cp2 = CartesianPose::Random("test");
  auto pose_vec = cp2.to_std_vector();
  cp1.set_data(pose_vec);
  for (std::size_t j = 0; j < pose_vec.size(); ++j) {
    EXPECT_FLOAT_EQ(pose_vec.at(j), cp1.data()(j));
  }
}

TEST(CartesianPoseTest, CartesianPoseToStdVector) {
  CartesianPose cp = CartesianPose::Random("test");
  std::vector<double> vec_data = cp.to_std_vector();
  EXPECT_EQ(vec_data.size(), 7);
  for (size_t i = 0; i < vec_data.size(); ++i) {
    EXPECT_EQ(cp.data()(i), vec_data[i]);
  }
}

TEST_F(CartesianPoseTestClass, MultiplyTransformsBothOperators) {
  CartesianPose tf3 = tf1 * tf2;
  tf1 *= tf2;
  EXPECT_EQ(tf3.get_name(), "t2");
  expect_near(tf1.get_position(), tf3.get_position());

  Eigen::Vector3d pos_truth(5, 7, 9);
  expect_near(tf1.get_position(), pos_truth);
}

TEST_F(CartesianPoseTestClass, MultiplyTransformsDifferentOrientation) {
  tf1.set_orientation(Eigen::Quaterniond(0.70710678, 0.70710678, 0., 0.));
  tf2.set_orientation(Eigen::Quaterniond(0., 0., 0.70710678, 0.70710678));
  tf1 *= tf2;
  Eigen::Vector3d pos_truth(5, -4, 8);
  Eigen::Quaterniond rot_truth(0., 0., 0., 1.);
  expect_near(tf1.get_position(), pos_truth);
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - tol);
}

TEST_F(CartesianPoseTestClass, TestInverseNullOrientation) {
  tf1 = tf1.inverse();
  Eigen::Vector3d pos_truth(-1, -2, -3);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);
  EXPECT_EQ(tf1.get_name(), "world");
  EXPECT_EQ(tf1.get_reference_frame(), "t1");
  expect_near(tf1.get_position(), pos_truth);
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - tol);
}

TEST_F(CartesianPoseTestClass, TestInverseNonNullOrientation) {
  tf1.set_orientation(Eigen::Quaterniond(0.70710678, 0.70710678, 0., 0.));
  tf1 = tf1.inverse();
  Eigen::Vector3d pos_truth(-1, -3, 2);
  Eigen::Quaterniond rot_truth(0.70710678, -0.70710678, 0., 0.);
  expect_near(tf1.get_position(), pos_truth);
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - tol);
}

TEST_F(CartesianPoseTestClass, TestMultiplyInverseNonNullOrientation) {
  tf1.set_orientation(Eigen::Quaterniond(0.70710678, 0.70710678, 0., 0.));
  tf1 *= tf1.inverse();
  Eigen::Vector3d pos_truth(0, 0, 0);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);
  expect_near(tf1.get_position(), pos_truth);
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - tol);
}

TEST(CartesianPoseTest, MultiplyPoseAndState) {
  CartesianPose p = CartesianPose::Random("test");
  CartesianState s = CartesianPose::Random("test2", "test");
  CartesianState res = p * s;
  CartesianPose res2 = p * static_cast<CartesianPose>(s);
  EXPECT_EQ(res2.get_type(), StateType::CARTESIAN_POSE);
  expect_near(res.get_position(), res2.get_position());
  EXPECT_GT(abs(res.get_orientation().dot(res2.get_orientation())), 1 - 1e-5);
}

TEST_F(CartesianPoseTestClass, TestAddTwoPoses) {
  tf1.set_position(Eigen::Vector3d::Zero());
  Eigen::Vector3d pos2(1, 0, 0);
  Eigen::Quaterniond rot2(0, 1, 0, 0);
  tf2 = CartesianPose("t1", pos2, rot2);
  auto tf3 = tf1 + tf2;
  EXPECT_EQ(tf3.get_type(), StateType::CARTESIAN_POSE);
  Eigen::Vector3d pos_truth(1, 0, 0);
  Eigen::Quaterniond rot_truth(0, -1, 0, 0);
  expect_near(tf3.get_position(), pos_truth);
  EXPECT_GT(abs(tf3.get_orientation().dot(rot_truth)), 1 - 1e-5);
}

TEST_F(CartesianPoseTestClass, TestPoseToVelocity) {
  tf1.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
  std::chrono::seconds dt1(1);
  std::chrono::milliseconds dt2(100);
  auto res1 = tf1 / dt1;
  EXPECT_EQ(res1.get_type(), StateType::CARTESIAN_TWIST);
  EXPECT_EQ(res1.get_linear_velocity(), tf1.get_position());
  EXPECT_EQ(res1.get_angular_velocity(), Eigen::Vector3d(M_PI, 0, 0));
  auto res2 = tf1 / dt2;
  EXPECT_EQ(res2.get_type(), StateType::CARTESIAN_TWIST);
  EXPECT_EQ(res2.get_linear_velocity(), 10 * tf1.get_position());
  EXPECT_EQ(res2.get_angular_velocity(), 10 * Eigen::Vector3d(M_PI, 0, 0));
}

TEST_F(CartesianPoseTestClass, TestImplicitConversion) {
  CartesianTwist vel("t1");
  EXPECT_EQ(vel.get_type(), StateType::CARTESIAN_TWIST);
  vel.set_linear_velocity(Eigen::Vector3d(0.1, 0.1, 0.1));
  vel.set_angular_velocity(Eigen::Vector3d(M_PI, 0, 0));
  tf1 += vel;
  EXPECT_EQ(tf1.get_type(), StateType::CARTESIAN_POSE);
  Eigen::Vector3d pos_truth(1.1, 2.1, 3.1);
  Eigen::Quaterniond rot_truth(0, 1, 0, 0);
  EXPECT_EQ(tf1.get_position(), pos_truth);
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - 1e-5);
}

TEST(CartesianPoseTest, TestPoseDistance) {
  CartesianPose p1(
      "test", Eigen::Vector3d(-0.353849997774433, -0.823586525156853, -1.57705702279920),
      Eigen::Quaterniond(-0.20495, -0.53709, -0.26785, -0.77316));
  CartesianPose p2(
      "test", Eigen::Vector3d(0.507974650905946, 0.281984063670556, 0.0334798822444514),
      Eigen::Quaterniond(-0.34616, 0.78063, 0.51161, 0.095041));
  EXPECT_FLOAT_EQ(p1.dist(p1), 0);
  EXPECT_FLOAT_EQ(dist(p1, p2), p1.dist(p2));
  EXPECT_NEAR(p1.dist(p2, CartesianStateVariable::POSITION), 2.13514804509215, 1e-3);
  EXPECT_NEAR(p1.dist(p2, CartesianStateVariable::ORIENTATION), 1.955608640373294, 1e-3);
  EXPECT_NEAR(p1.dist(p2), 4.090756685465443, 1e-3);
}

TEST(CartesianPoseTest, TestFilter) {
  CartesianPose tf1("t1", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  CartesianPose tf2("t1", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  for (int i = 0; i < 1000; ++i) {
    CartesianPose temp = tf1;
    double alpha = 0.1;
    tf1 = (1 - alpha) * tf1 + alpha * tf2;
    EXPECT_LT((tf1.get_position() - ((1 - alpha) * temp.get_position() + alpha * tf2.get_position())).norm(), 1e-4);
  }
}

TEST(CartesianPoseTest, TestToSTDVector) {
  CartesianPose p = CartesianPose::Random("t1");
  std::vector<double> v = p.to_std_vector();
  for (unsigned int i = 0; i < 3; ++i) {
    EXPECT_NEAR(p.get_position()(i), v[i], 0.00001);
  }
  EXPECT_NEAR(p.get_orientation().w(), v[3], 0.00001);
  EXPECT_NEAR(p.get_orientation().x(), v[4], 0.00001);
  EXPECT_NEAR(p.get_orientation().y(), v[5], 0.00001);
  EXPECT_NEAR(p.get_orientation().z(), v[6], 0.00001);
}

TEST(CartesianPoseTest, TestPoseNorms) {
  std::vector<double> norms;
  double tolerance = 1e-4;
  CartesianState cs = CartesianState::Random("cs");
  // independent variables first
  norms = cs.norms(CartesianStateVariable::POSITION);
  EXPECT_TRUE(norms.size() == 1);
  EXPECT_NEAR(norms[0], cs.get_position().norm(), tolerance);
  norms = cs.norms(CartesianStateVariable::ORIENTATION);
  EXPECT_TRUE(norms.size() == 1);
  EXPECT_NEAR(norms[0], cs.get_orientation().norm(), tolerance);
  // then grouped by two
  norms = cs.norms(CartesianStateVariable::POSE);
  EXPECT_TRUE(norms.size() == 2);
  EXPECT_NEAR(norms[0], cs.get_position().norm(), tolerance);
  EXPECT_NEAR(norms[1], cs.get_orientation().norm(), tolerance);
  // test with CartesianPose default variable
  CartesianPose cp = CartesianPose::Random("cp");
  std::vector<double> pose_norms = cp.norms();
  EXPECT_TRUE(pose_norms.size() == 2);
  EXPECT_NEAR(pose_norms[0], cp.get_position().norm(), tolerance);
  EXPECT_NEAR(pose_norms[1], cp.get_orientation().norm(), tolerance);
}
