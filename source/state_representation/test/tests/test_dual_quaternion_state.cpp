#include "state_representation/space/dual_quaternion/DualQuaternionPose.hpp"
#include <gtest/gtest.h>

TEST(DualQuaternionStateTest, MultiplyTransformsBothOperators) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1, 0, 0, 0);
  state_representation::DualQuaternionPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(1, 0, 0, 0);
  state_representation::DualQuaternionPose tf2("t2", pos2, rot2);

  state_representation::DualQuaternionPose tf3 = tf1 * tf2;
  tf1 *= tf2;

  EXPECT_EQ(tf3.get_name(), "t2");
  for (int i = 0; i < tf1.get_position().size(); ++i)
    EXPECT_NEAR(tf1.get_position()(i),
                tf3.get_position()(i),
                0.00001);
}

TEST(DualQuaternionStateTest, MultiplyTransformsSameOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1, 0, 0, 0);
  state_representation::DualQuaternionPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(1, 0, 0, 0);
  state_representation::DualQuaternionPose tf2("t2", pos2, rot2);

  tf1 *= tf2;

  Eigen::Vector3d pos_truth(5, 7, 9);
  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
}

TEST(DualQuaternionStateTest, MultiplyTransformsDifferentOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  state_representation::DualQuaternionPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(0., 0., 0.70710678, 0.70710678);
  state_representation::DualQuaternionPose tf2("t2", pos2, rot2);

  tf1 *= tf2;

  Eigen::Vector3d pos_truth(5, -4, 8);
  Eigen::Quaterniond rot_truth(0., 0., 0., 1.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(DualQuaternionStateTest, TestInverseNullOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1., 0., 0., 0.);
  state_representation::DualQuaternionPose tf1("t1", pos1, rot1);

  tf1 = tf1.inverse();

  Eigen::Vector3d pos_truth(-1, -2, -3);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  EXPECT_EQ(tf1.get_name(), "world");
  EXPECT_EQ(tf1.get_reference_frame(), "t1");
  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(DualQuaternionStateTest, TestInverseNonNullOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  state_representation::DualQuaternionPose tf1("t1", pos1, rot1);

  tf1 = tf1.inverse();

  Eigen::Vector3d pos_truth(-1, -3, 2);
  Eigen::Quaterniond rot_truth(0.70710678, -0.70710678, 0., 0.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}

TEST(DualQuaternionStateTest, TestMultiplyInverseNonNullOrientation) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  state_representation::DualQuaternionPose tf1("t1", pos1, rot1);

  tf1 *= tf1.inverse();

  Eigen::Vector3d pos_truth(0, 0, 0);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(tf1.get_orientation().coeffs()(i), rot_truth.coeffs()(i), 0.00001);
}
