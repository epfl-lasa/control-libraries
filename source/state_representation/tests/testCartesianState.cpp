#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <unistd.h>

using namespace StateRepresentation;

TEST(NegateQuaternion, PositiveNos) {
  Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
  Eigen::Quaterniond q2 = Eigen::Quaterniond(-q.coeffs());

  EXPECT_TRUE(q.w() == -q2.w());
  for (int i = 0; i < 3; ++i) EXPECT_TRUE(q.vec()(i) == -q2.vec()(i));
}

TEST(MultiplyTransformsBothOperators, PositiveNos) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1, 0, 0, 0);
  CartesianPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(1, 0, 0, 0);
  CartesianPose tf2("t2", pos2, rot2, "t1");

  CartesianPose tf3 = tf1 * tf2;
  tf1 *= tf2;

  EXPECT_EQ(tf3.get_name(), "t2");
  for (int i = 0; i < tf1.get_position().size(); ++i) EXPECT_NEAR(tf1.get_position()(i), tf3.get_position()(i), 0.00001);
}

TEST(MultiplyTransformsSameOrientation, PositiveNos) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1, 0, 0, 0);
  CartesianPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(1, 0, 0, 0);
  CartesianPose tf2("t2", pos2, rot2, "t1");

  tf1 *= tf2;

  Eigen::Vector3d pos_truth(5, 7, 9);
  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
}

TEST(MultiplyTransformsDifferentOrientation, PositiveNos) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(4, 5, 6);
  Eigen::Quaterniond rot2(0., 0., 0.70710678, 0.70710678);
  CartesianPose tf2("t2", pos2, rot2, "t1");

  tf1 *= tf2;

  Eigen::Vector3d pos_truth(5, -4, 8);
  Eigen::Quaterniond rot_truth(0., 0., 0., 1.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1 - 10E-4);
}

TEST(TestInverseNullOrientation, PositiveNos) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(1., 0., 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);

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
  EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1 - 10E-4);
}

TEST(TestInverseNonNullOrientation, PositiveNos) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);

  tf1 = tf1.inverse();

  Eigen::Vector3d pos_truth(-1, -3, 2);
  Eigen::Quaterniond rot_truth(0.70710678, -0.70710678, 0., 0.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1 - 10E-4);
}

TEST(TestMultiplyInverseNonNullOrientation, PositiveNos) {
  Eigen::Vector3d pos1(1, 2, 3);
  Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
  CartesianPose tf1("t1", pos1, rot1);

  tf1 *= tf1.inverse();

  Eigen::Vector3d pos_truth(0, 0, 0);
  Eigen::Quaterniond rot_truth(1., 0., 0., 0.);

  std::cerr << "position" << std::endl;
  std::cerr << tf1.get_position() << std::endl;
  std::cerr << "orientation" << std::endl;
  std::cerr << tf1.get_orientation().coeffs() << std::endl;

  for (int i = 0; i < pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
  EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1 - 10E-4);
}

TEST(MultiplyPoseAndState, PositiveNos) {
  CartesianPose p = CartesianPose::Random("test");
  CartesianState s = CartesianPose::Random("test2", "test");

  CartesianState res = p * s;
  CartesianPose res2 = p * static_cast<CartesianPose>(s);

  for (int i = 0; i < res.get_position().size(); ++i) EXPECT_NEAR(res.get_position()(i), res2.get_position()(i), 0.00001);
  EXPECT_TRUE(abs(res.get_orientation().dot(res2.get_orientation())) > 1 - 10E-4);
}

TEST(TestAddTwoPoses, PositiveNos) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(1, 0, 0);
  Eigen::Quaterniond rot2(0, 1, 0, 0);
  CartesianPose tf2("t1", pos2, rot2);

  std::cout << tf1 + tf2 << std::endl;
  std::cout << tf1 - tf2 << std::endl;
}

TEST(TestAddDisplacement, PositiveNos) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);

  CartesianTwist vel("t1");
  vel.set_linear_velocity(Eigen::Vector3d(0.1, 0.1, 0.1));
  vel.set_angular_velocity(Eigen::Vector3d(0.1, 0.1, 0));

  std::chrono::milliseconds dt1(10);
  std::cout << tf1 + dt1 * vel << std::endl;

  std::chrono::milliseconds dt2(1000);
  std::cout << tf1 + dt2 * vel << std::endl;

  std::chrono::seconds dt3(1);
  std::cout << tf1 + dt3 * vel << std::endl;
}

TEST(TestPoseToVelocity, PositiveNos) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);

  Eigen::Vector3d pos2(1, 0, 0);
  Eigen::Quaterniond rot2(0, 1, 0, 0);
  CartesianPose tf2("t1", pos2, rot2);

  std::chrono::seconds dt1(1);
  std::cout << (tf1 - tf2) / dt1 << std::endl;

  std::chrono::seconds dt2(10);
  std::cout << (tf1 - tf2) / dt2 << std::endl;

  std::chrono::milliseconds dt3(100);
  std::cout << (tf1 - tf2) / dt3 << std::endl;
}

TEST(TestTwistStateMultiplication, PositiveNos) {
  CartesianTwist twist = CartesianTwist::Random("test");
  CartesianState state = CartesianTwist::Random("test2", "test");

  std::cout << twist * state << std::endl;
}

TEST(TestImplicitConversion, PositiveNos) {
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity();
  CartesianPose tf1("t1", pos1, rot1);

  CartesianTwist vel("t1");
  vel.set_linear_velocity(Eigen::Vector3d(0.1, 0.1, 0.1));
  vel.set_angular_velocity(Eigen::Vector3d(0.1, 0.1, 0));

  tf1 += vel;

  std::cout << tf1 << std::endl;
}

TEST(TestVelocityClamping, PositiveNos) {
  CartesianTwist vel("test", Eigen::Vector3d(1, -2, 3), Eigen::Vector3d(1, 2, -3));
  vel.clamp(1, 0.5);

  std::cout << vel << std::endl;
  EXPECT_TRUE(vel.get_linear_velocity().norm() <= 1);
  EXPECT_TRUE(vel.get_angular_velocity().norm() <= 0.5);

  vel *= 0.01;

  std::cout << vel.clamped(1, 0.5, 0.1, 0.1) << std::endl;
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(vel.clamped(1, 0.5, 0.1, 0.1).get_linear_velocity()(i) == 0);
    EXPECT_TRUE(vel.clamped(1, 0.5, 0.1, 0.1).get_angular_velocity()(i) == 0);
  }
}

TEST(TestPoseDistance, PositiveNos) {
  CartesianPose p1("test", Eigen::Vector3d::Zero());
  CartesianPose p2("test", Eigen::Vector3d(1, 0, 0));
  CartesianPose p3("test", Eigen::Vector3d(1, 0, 0), Eigen::Quaterniond(0, 1, 0, 0));

  double d1 = dist(p1, p2);
  double d2 = p1.dist(p2);

  //EXPECT_TRUE(abs(d1 - d2) < 1e-4);
  //EXPECT_TRUE(d1 < 1e-4);

  double d3 = dist(p1, p3);
  //EXPECT_TRUE(abs(d3 - 3.14159) < 1e10-3);
}

TEST(TestFilter, PositiveNos) {
  CartesianPose tf1("t1", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
  CartesianPose tf2("t1", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

  for (int i = 0; i < 1000; ++i) {
    CartesianPose temp = tf1;

    double alpha = 0.1;
    tf1 = (1 - alpha) * tf1 + alpha * tf2;
    EXPECT_TRUE((tf1.get_position() - ((1 - alpha) * temp.get_position() + alpha * tf2.get_position())).norm() < 1e-4);
  }

  //EXPECT_TRUE(dist(tf1, tf2) < 1e-4);
}

TEST(TestToSTDVector, PositiveNos) {
  CartesianPose p = CartesianPose::Random("t1");
  std::vector<double> v = p.to_std_vector();
  for (unsigned int i = 0; i < 3; ++i) EXPECT_NEAR(p.get_position()(i), v[i], 0.00001);
  EXPECT_NEAR(p.get_orientation().w(), v[3], 0.00001);
  EXPECT_NEAR(p.get_orientation().x(), v[4], 0.00001);
  EXPECT_NEAR(p.get_orientation().y(), v[5], 0.00001);
  EXPECT_NEAR(p.get_orientation().z(), v[6], 0.00001);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
