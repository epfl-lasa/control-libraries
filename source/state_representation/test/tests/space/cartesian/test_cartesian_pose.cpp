#include <gtest/gtest.h>

#include "state_representation/space/cartesian/CartesianPose.hpp"

using namespace state_representation;

TEST(CartesianPoseTest, RandomPoseInitialization) {
  CartesianPose random = CartesianPose::Random("test");
  // only position should be random
  EXPECT_GT(random.get_position().norm(), 0);
  EXPECT_GT(abs(random.get_orientation().w()), 0);
  EXPECT_GT(abs(random.get_orientation().x()), 0);
  EXPECT_GT(abs(random.get_orientation().y()), 0);
  EXPECT_GT(abs(random.get_orientation().z()), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(random).get_wrench().norm(), 0);
}

TEST(CartesianPoseTest, CopyPose) {
  CartesianPose pose1 = CartesianPose::Random("test");
  CartesianPose pose2(pose1);
  EXPECT_EQ(pose1.get_name(), pose2.get_name());
  EXPECT_EQ(pose1.get_reference_frame(), pose2.get_reference_frame());
  EXPECT_TRUE(pose1.data().isApprox(pose2.data()));
  EXPECT_EQ(static_cast<CartesianState&>(pose2).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose2).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose2).get_wrench().norm(), 0);
  CartesianPose pose3 = pose1;
  EXPECT_EQ(pose1.get_name(), pose3.get_name());
  EXPECT_EQ(pose1.get_reference_frame(), pose3.get_reference_frame());
  EXPECT_TRUE(pose1.data().isApprox(pose3.data()));
  EXPECT_EQ(static_cast<CartesianState&>(pose3).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose3).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose3).get_wrench().norm(), 0);
  // try to change non pose variables prior to the copy, those should be discarded
  static_cast<CartesianState&>(pose1).set_twist(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(pose1).set_accelerations(Eigen::VectorXd::Random(6));
  static_cast<CartesianState&>(pose1).set_wrench(Eigen::VectorXd::Random(6));
  CartesianPose pose4 = pose1;
  EXPECT_TRUE(pose1.data().isApprox(pose4.data()));
  EXPECT_EQ(static_cast<CartesianState&>(pose4).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose4).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose4).get_wrench().norm(), 0);
  // copy a state, only the pose variables should be non 0
  CartesianPose pose5 = CartesianState::Random("test");
  EXPECT_EQ(static_cast<CartesianState&>(pose5).get_twist().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose5).get_accelerations().norm(), 0);
  EXPECT_EQ(static_cast<CartesianState&>(pose5).get_wrench().norm(), 0);

  CartesianPose pose6;
  EXPECT_TRUE(pose6.is_empty());
  CartesianPose pose7 = pose6;
  EXPECT_TRUE(pose7.is_empty());
}

TEST(CartesianPoseTest, SetData) {
  CartesianPose cp1 = CartesianPose::Identity("test");
  CartesianPose cp2 = CartesianPose::Random("test");
  cp1.set_data(cp2.data());
  EXPECT_TRUE(cp2.data().isApprox(cp1.data()));

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

TEST(CartesianPoseTest, MultiplyTransformsBothOperators) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1, 0, 0, 0);
  CartesianPose tf1("t1", pos1, rot1);
  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(1, 0, 0, 0);
  CartesianPose tf2("t2", pos2, rot2, "t1");
  CartesianPose tf3 = tf1 * tf2;
  tf1 *= tf2;
  EXPECT_EQ(tf3.get_name(), "t2");
  for (int i = 0; i < tf1.get_position().size(); ++i)
    EXPECT_NEAR(tf1.get_position()(i), tf3.get_position()(i), 0.00001);
}

TEST(CartesianPoseTest, MultiplyTransformsSameOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1, 0, 0, 0);
  CartesianPose tf1("t1", pos1, rot1);
  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(1, 0, 0, 0);
  CartesianPose tf2("t2", pos2, rot2, "t1");
  tf1 *= tf2;
  Eigen::Vector3d pos_truth(5, 7, 9);
  for (int i = 0; i < pos_truth.size(); ++i) {
    EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  }
}

TEST(CartesianPoseTest, MultiplyTransformsDifferentOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);
  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(0., 0., 0.70710678, 0.70710678);
  CartesianPose tf2("t2", pos2, rot2, "t1");
  tf1 *= tf2;
  Eigen::Vector3d pos_truth(5, -4, 8);
  Eigen::Quaterniond rot_truth(0., 0., 0., 1.);
  for (int i = 0; i < pos_truth.size(); ++i) {
    EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  }
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - 10E-4);
}

TEST(CartesianPoseTest, TestInverseNullOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1., 0., 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);
  tf1 = tf1.inverse();
  Eigen::Vector3d pos_truth(-1, -2, -3);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);
  EXPECT_EQ(tf1.get_name(), "world");
  EXPECT_EQ(tf1.get_reference_frame(), "t1");
  for (int i = 0; i < pos_truth.size(); ++i) {
    EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  }
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - 10E-4);
}

TEST(CartesianPoseTest, TestInverseNonNullOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);
  tf1 = tf1.inverse();
  Eigen::Vector3d pos_truth(-1, -3, 2);
  Eigen::Quaterniond rot_truth(0.70710678, -0.70710678, 0., 0.);
  for (int i = 0; i < pos_truth.size(); ++i) {
    EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  }
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - 10E-4);
}

TEST(CartesianPoseTest, TestMultiplyInverseNonNullOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);
  tf1 *= tf1.inverse();
  Eigen::Vector3d pos_truth(0, 0, 0);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);
  for (int i = 0; i < pos_truth.size(); ++i) {
    EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  }
  EXPECT_GT(abs(tf1.get_orientation().dot(rot_truth)), 1 - 10E-4);
}

TEST(CartesianPoseTest, MultiplyPoseAndState) {
  CartesianPose p = CartesianPose::Random("test");
  CartesianState s = CartesianPose::Random("test2", "test");
  CartesianState res = p * s;
  CartesianPose res2 = p * static_cast<CartesianPose>(s);
  for (int i = 0; i < res.get_position().size(); ++i) {
    EXPECT_NEAR(res.get_position()(i), res2.get_position()(i), 0.00001);
  }
  EXPECT_GT(abs(res.get_orientation().dot(res2.get_orientation())), 1 - 10E-4);
}

TEST(CartesianPoseTest, TestAddTwoPoses) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);
  Eigen::Vector3d pos2(1, 0, 0);
  Eigen::Quaterniond rot2(0, 1, 0, 0);
  CartesianPose tf2("t1", pos2, rot2);
}

TEST(CartesianPoseTest, TestAddDisplacement) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);
  CartesianTwist vel("t1");
  vel.set_linear_velocity(Eigen::Vector3d(0.1, 0.1, 0.1));
  vel.set_angular_velocity(Eigen::Vector3d(0.1, 0.1, 0));
  std::chrono::milliseconds dt1(10);
  std::chrono::milliseconds dt2(1000);
  std::chrono::seconds dt3(1);
}

TEST(CartesianPoseTest, TestPoseToVelocity) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);
  Eigen::Vector3d pos2(1, 0, 0);
  Eigen::Quaterniond rot2(0, 1, 0, 0);
  CartesianPose tf2("t1", pos2, rot2);
  std::chrono::seconds dt1(1);
  std::chrono::seconds dt2(10);
  std::chrono::milliseconds dt3(100);
}

TEST(CartesianPoseTest, TestImplicitConversion) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);
  CartesianTwist vel("t1");
  vel.set_linear_velocity(Eigen::Vector3d(0.1, 0.1, 0.1));
  vel.set_angular_velocity(Eigen::Vector3d(0.1, 0.1, 0));
  tf1 += vel;
}

TEST(CartesianPoseTest, TestPoseDistance) {
  CartesianPose p1("test",
                   Eigen::Vector3d(-0.353849997774433, -0.823586525156853, -1.57705702279920),
                   Eigen::Quaterniond(-0.20495, -0.53709, -0.26785, -0.77316));
  CartesianPose p2("test",
                   Eigen::Vector3d(0.507974650905946, 0.281984063670556, 0.0334798822444514),
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
