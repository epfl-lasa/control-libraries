#include "controllers/impedance/Impedance.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>

using namespace controllers::impedance;
using namespace StateRepresentation;

TEST(TestCartesianImpedance, PositiveNos) {
  Impedance<CartesianState> impedance_controller(Eigen::MatrixXd::Identity(6, 6),
                                                 Eigen::MatrixXd::Identity(6, 6),
                                                 Eigen::MatrixXd::Identity(6, 6));
  // set up a desired state6d
  CartesianState desired_state("test");
  desired_state.set_linear_velocity(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  CartesianState feedback_state("test");
  feedback_state.set_orientation(Eigen::Quaterniond(0, 0, 1, 1));
  feedback_state.set_linear_velocity(Eigen::Vector3d(0.5, 0, 0));
  // check command
  CartesianWrench command = impedance_controller.compute_command(desired_state, feedback_state);

  std::cerr << command << std::endl;
}

TEST(TestJointImpedance, PositiveNos) {
  int nb_joints = 3;
  Impedance<JointState> impedance_controller(Eigen::MatrixXd::Identity(nb_joints, nb_joints),
                                             Eigen::MatrixXd::Identity(nb_joints, nb_joints),
                                             Eigen::MatrixXd::Identity(nb_joints, nb_joints));
  // set up a desired state6d
  JointState desired_state("test", nb_joints);
  desired_state.set_velocities(Eigen::Vector3d(1, 0, 0));
  // set up a real state
  JointState feedback_state("test", nb_joints);
  feedback_state.set_positions(Eigen::Vector3d(0, 0, 1));
  feedback_state.set_velocities(Eigen::Vector3d(0.5, 0, 0));
  // check command
  JointTorques command = impedance_controller.compute_command(desired_state, feedback_state);

  std::cerr << command << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
