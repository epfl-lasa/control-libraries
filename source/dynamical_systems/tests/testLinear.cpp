#include "dynamical_systems/Linear.hpp"
#include <gtest/gtest.h>
#include <unistd.h>
#include <vector>

TEST(EvaluateDynamicalSystemPositionOnly, PositiveNos) {
  state_representation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
  state_representation::CartesianPose target_pose("robot", 10 * Eigen::Vector3d::Random());
  DynamicalSystems::Linear<state_representation::CartesianState> linearDS(target_pose);

  unsigned int nb_steps = 100;
  double dt = 0.1;

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }

  std::cout << current_pose << std::endl;
  std::cout << target_pose << std::endl;
  std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

  for (int i = 0; i < 3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.001);
  EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1 - 10E-4);
}

TEST(EvaluateDynamicalSystemOrientationOnly, PositiveNos) {
  srand(time(NULL));

  state_representation::CartesianPose current_pose("robot", Eigen::Vector3d(0, 0, 0));
  state_representation::CartesianPose target_pose("robot", Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::UnitRandom());

  DynamicalSystems::Linear<state_representation::CartesianState> linearDS(target_pose);

  unsigned int nb_steps = 100;
  double dt = 0.1;

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }

  std::cout << current_pose << std::endl;
  std::cout << target_pose << std::endl;
  std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

  for (int i = 0; i < 3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
  EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1 - 10E-4);
}

TEST(EvaluateDynamicalSystem, PositiveNos) {
  srand(time(NULL));

  state_representation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
  state_representation::CartesianPose target_pose("robot", 10 * Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

  DynamicalSystems::Linear<state_representation::CartesianState> linearDS(target_pose);

  unsigned int nb_steps = 100;
  double dt = 0.1;

  for (unsigned int i = 0; i < nb_steps; ++i) {
    state_representation::CartesianTwist twist = linearDS.evaluate(current_pose);
    current_pose += dt * twist;
  }

  std::cout << current_pose << std::endl;
  std::cout << target_pose << std::endl;
  std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

  for (int i = 0; i < 3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
  EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1 - 10E-4);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}