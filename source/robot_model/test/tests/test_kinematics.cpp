#include "robot_model/Model.hpp"

#include <stdexcept>
#include <memory>
#include <gtest/gtest.h>

#include "robot_model/exceptions/InvalidJointStateSizeException.hpp"
#include "robot_model/exceptions/FrameNotFoundException.hpp"
#include "robot_model/exceptions/InverseKinematicsNotConvergingException.hpp"

using namespace robot_model;

class RobotModelKinematicsTest : public testing::Test {
protected:
  void SetUp() override {
    robot_name = "franka";
    urdf_path = std::string(TEST_FIXTURES) + "panda_arm.urdf";
    franka = std::make_unique<Model>(robot_name, urdf_path);
    joint_state = state_representation::JointState(robot_name, franka->get_joint_frames());
    set_test_configurations();
  }

  std::unique_ptr<Model> franka;
  std::string robot_name;
  std::string urdf_path;

  state_representation::JointState joint_state;

  double tol = 1e-5;

  // Known test configurations generated by fixtures/generateRobotModelKinematicsTestConfigurations.m
  std::vector<state_representation::JointState> test_configs;
  std::vector<Eigen::MatrixXd> test_jacobian_ee_expects;
  std::vector<state_representation::CartesianPose> test_fk_ee_expects;
  std::vector<state_representation::CartesianPose> test_fk_link4_expects;
  std::vector<state_representation::CartesianTwist> test_velocity_fk_expects;
  Eigen::Matrix<double, 6, 1> twist;

  void set_test_configurations() {
    // Random test configuration 1:
    state_representation::JointState config1(franka->get_robot_name(), franka->get_joint_frames());
    config1.set_positions(std::vector<double>{-1.957518, 1.037530, -1.093933, -1.485144, -1.937432, 2.251972, -1.373487});
    config1.set_velocities(std::vector<double>{0.308158, 0.378429, 0.496303, -0.098917, -0.832357, -0.542046, 0.826675});
    test_configs.push_back(config1);

    // Expected results for configuration 1:
    Eigen::MatrixXd jac1(6, 7);
    jac1 << 0.275154, -0.005914, 0.127368, -0.041238, 0.003824, 0.018324, 0.000000,
        -0.693174, -0.014523, -0.347282, -0.422943, 0.026691, -0.034385, 0.000000,
        -0.000000, -0.516268, -0.463478, 0.313197, -0.006376, -0.132947, 0.000000,
        -0.000000, 0.926150, -0.324787, -0.254761, -0.935275, -0.138023, -0.842118,
        0.000000, -0.377155, -0.797555, 0.591396, 0.050315, -0.963323, 0.236446,
        1.000000, 0.000000, 0.508349, 0.765080, -0.350326, 0.230127, 0.484697;
    test_jacobian_ee_expects.emplace_back(jac1);
    test_fk_ee_expects.emplace_back(state_representation::CartesianPose("ee",
                                                                        Eigen::Vector3d(-0.693174, -0.275154, 0.348681),
                                                                        Eigen::Quaterniond(0.548764,
                                                                                           -0.464146,
                                                                                           -0.205476,
                                                                                           0.664234),
                                                                        franka->get_base_frame()));
    test_fk_link4_expects.emplace_back(state_representation::CartesianPose("link4",
                                                                           Eigen::Vector3d(-0.177776,
                                                                                           -0.242212,
                                                                                           0.461029),
                                                                           Eigen::Quaterniond(0.717822,
                                                                                              -0.327979,
                                                                                              0.099446,
                                                                                              0.606030),
                                                                           franka->get_base_frame()));
    twist << 0.136730, -0.353202, -0.379006, 0.371630, 0.078694, 1.052318;
    test_velocity_fk_expects.emplace_back(state_representation::CartesianTwist("ee", twist, franka->get_base_frame()));

    // Random test configuration 2:
    state_representation::JointState config2(franka->get_robot_name(), franka->get_joint_frames());
    config2.set_positions(std::vector<double>{-2.014330, 1.148700, 0.222179, -0.081404, -2.444304, 1.651397, -2.279290});
    config2.set_velocities(std::vector<double>{0.923796, -0.990732, 0.549821, 0.634606, 0.737389, -0.831128, -0.200435});
    test_configs.push_back(config2);

    // Expected results for configuration 2:
    Eigen::MatrixXd jac2(6, 7);
    jac2 << 0.627192, -0.150505, -0.032035, 0.033135, -0.066413, 0.022648, -0.000000,
        -0.375992, -0.316782, -0.016739, 0.322201, 0.000983, -0.134882, -0.000000,
        0.000000, -0.727856, -0.064277, 0.388627, -0.074176, -0.022065, -0.000000,
        0.000000, 0.903241, -0.391470, -0.919778, -0.387933, 0.667016, -0.665249,
        -0.000000, -0.429134, -0.823965, 0.337047, -0.858275, -0.009872, 0.442326,
        1.000000, 0.000000, 0.409673, -0.201016, 0.335963, 0.744978, 0.601491;
    test_jacobian_ee_expects.emplace_back(jac2);
    test_fk_ee_expects.emplace_back(state_representation::CartesianPose("ee",
                                                                        Eigen::Vector3d(-0.375992, -0.627192, 0.683717),
                                                                        Eigen::Quaterniond(0.399601,
                                                                                           -0.442959,
                                                                                           0.055149,
                                                                                           0.800665),
                                                                        franka->get_base_frame()));
    test_fk_link4_expects.emplace_back(state_representation::CartesianPose("link4",
                                                                           Eigen::Vector3d(-0.121432,
                                                                                           -0.297952,
                                                                                           0.389048),
                                                                           Eigen::Quaterniond(0.000372,
                                                                                              -0.727768,
                                                                                              0.266200,
                                                                                              0.632054),
                                                                           franka->get_base_frame()));
    twist << 0.664126, 0.274603, 0.896037, -2.400900, -0.527320, 0.529481;
    test_velocity_fk_expects.emplace_back(state_representation::CartesianTwist("ee", twist, franka->get_base_frame()));

    // Random test configuration 3:
    state_representation::JointState config3(franka->get_robot_name(), franka->get_joint_frames());
    config3.set_positions(std::vector<double>{-1.391455, 1.057921, -0.397429, -0.338036, -1.843569, 0.977037, -2.053960});
    config3.set_velocities(std::vector<double>{-0.727863, 0.738584, 0.159409, 0.099720, -0.710090, 0.706062, 0.244110});
    test_configs.push_back(config3);

    // Expected results for configuration 3:
    Eigen::MatrixXd jac3(6, 7);
    jac3 << 0.675235, 0.057394, 0.055473, 0.127263, -0.079424, 0.017167, -0.000000,
        -0.041899, -0.316587, -0.070569, 0.196918, -0.025838, -0.136065, 0.000000,
        0.000000, -0.656931, -0.140875, 0.342071, -0.109756, 0.019609, -0.000000,
        -0.000000, 0.983961, 0.155431, -0.941148, 0.047099, 0.575869, -0.702840,
        0.000000, 0.178382, -0.857362, 0.022395, -0.979378, 0.187336, 0.610654,
        1.000000, 0.000000, 0.490684, 0.337251, 0.196472, 0.795789, 0.364853;
    test_jacobian_ee_expects.emplace_back(jac3);
    test_fk_ee_expects.emplace_back(state_representation::CartesianPose("ee",
                                                                        Eigen::Vector3d(-0.041899, -0.675235, 0.654747),
                                                                        Eigen::Quaterniond(0.264829,
                                                                                           -0.521437,
                                                                                           0.213722,
                                                                                           0.782491),
                                                                        franka->get_base_frame()));
    test_fk_link4_expects.emplace_back(state_representation::CartesianPose("link4",
                                                                           Eigen::Vector3d(0.024355,
                                                                                           -0.313350,
                                                                                           0.421774),
                                                                           Eigen::Quaterniond(0.076121,
                                                                                              0.571715,
                                                                                              -0.067208,
                                                                                              -0.814144),
                                                                           franka->get_base_frame()));
    twist << -0.359035, -0.272665, -0.381763, 0.859248, 0.974096, -0.104585;
    test_velocity_fk_expects.emplace_back(state_representation::CartesianTwist("ee", twist, franka->get_base_frame()));
  }
};

TEST_F(RobotModelKinematicsTest, TestForwardKinematicsJointStateSize) {
  state_representation::JointState dummy = state_representation::JointState(robot_name, 6);
  EXPECT_THROW(franka->forward_kinematics(dummy), exceptions::InvalidJointStateSizeException);
}

TEST_F(RobotModelKinematicsTest, TestForwardKinematicsEE) {
  EXPECT_EQ(franka->forward_kinematics(joint_state).get_position(),
            franka->forward_kinematics(joint_state, "panda_link8").get_position());
}

TEST_F(RobotModelKinematicsTest, TestForwardKinematicsInvalidFrameName) {
  EXPECT_THROW(franka->forward_kinematics(joint_state, "panda_link99"), exceptions::FrameNotFoundException);
}

TEST_F(RobotModelKinematicsTest, TestForwardKinematics) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::CartesianPose ee_pose = franka->forward_kinematics(test_configs[config]);
    EXPECT_LT(ee_pose.dist(test_fk_ee_expects.at(config)), 1e-3);
    state_representation::CartesianPose link4_pose = franka->forward_kinematics(test_configs[config], "panda_link4");
    EXPECT_LT(link4_pose.dist(test_fk_link4_expects.at(config)), 1e-3);
  }
}

TEST_F(RobotModelKinematicsTest, TestForwardVelocity) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::CartesianTwist ee_twist = franka->forward_velocity(test_configs[config]);
    EXPECT_LT(ee_twist.dist(test_velocity_fk_expects.at(config)), 1e-3);
  }
}

TEST_F(RobotModelKinematicsTest, TestInverseVelocity) {
  std::string eef_frame = franka->get_frames().back();
  for (auto& config : test_configs) {
    state_representation::CartesianTwist des_ee_twist = state_representation::CartesianTwist::Random(eef_frame,
                                                                                                     franka->get_base_frame());
    state_representation::JointVelocities joint_velocities = franka->inverse_velocity(des_ee_twist, config);

    state_representation::JointState state(config);
    state.set_velocities(joint_velocities.data());
    state_representation::CartesianTwist act_ee_twist = franka->forward_velocity(state);

    EXPECT_LT(des_ee_twist.dist(act_ee_twist), 1e-3);

    // second method call the QP based inverse velocity
    state_representation::JointVelocities joint_velocities2 = franka->inverse_velocity(des_ee_twist,
                                                                                       config,
                                                                                       QPInverseVelocityParameters());

    state_representation::JointState state2(config);
    state.set_velocities(joint_velocities2.data());
    state_representation::CartesianTwist act_ee_twist2 = franka->forward_velocity(state);
  }
}

TEST_F(RobotModelKinematicsTest, TestInverseVelocityConstraints) {
  std::string eef_frame = franka->get_frames().back();
  QPInverseVelocityParameters parameters;
  parameters.linear_velocity_limit = 0.1;
  parameters.angular_velocity_limit = 0.2;
  for (auto& config : test_configs) {
    state_representation::CartesianTwist des_ee_twist(eef_frame,
                                                      Eigen::Vector3d::Identity(),
                                                      Eigen::Vector3d::Identity(),
                                                      franka->get_base_frame());
    std::cout << des_ee_twist << std::endl;

    state_representation::JointVelocities joint_velocities = franka->inverse_velocity(des_ee_twist, config, parameters);

    state_representation::JointState state(config);
    state.set_velocities(joint_velocities.data());
    state_representation::CartesianTwist act_ee_twist = franka->forward_velocity(state);

    EXPECT_LE(act_ee_twist.get_linear_velocity().norm(), 0.1);
    EXPECT_LE(act_ee_twist.get_angular_velocity().norm(), 0.2);
  }
}

TEST_F(RobotModelKinematicsTest, TestInRange) {
  state_representation::JointPositions joint_positions("robot", franka->get_joint_frames());
  state_representation::JointVelocities joint_velocities("robot", franka->get_joint_frames());
  state_representation::JointTorques joint_torques("robot", franka->get_joint_frames());
  state_representation::JointState joint_state("robot", franka->get_joint_frames());

  joint_positions.set_positions(std::vector<double>{2.648782, -0.553976, 0.801067, -2.042097, -1.642935, 2.946476, 1.292717});
  joint_velocities.set_velocities(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  joint_torques.set_torques(std::vector<double>{-0.329909, -0.235174, -1.881858, -2.491807, 0.674615, 0.996670, 0.345810});

  joint_state.set_positions(std::vector<double>{2.648782, -0.553976, 0.801067, -2.042097, -1.642935, 2.946476, 1.292717});
  joint_state.set_velocities(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  joint_state.set_torques(std::vector<double>{-0.329909, -0.235174, -1.881858, -2.491807, 0.674615, 0.996670, 0.345810});

  EXPECT_TRUE(franka->in_range(joint_positions));
  EXPECT_TRUE(franka->in_range(joint_velocities));
  EXPECT_TRUE(franka->in_range(joint_torques));
  EXPECT_TRUE(franka->in_range(joint_state));

  joint_positions.set_positions(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 1000});
  joint_state.set_positions(std::vector<double>{-0.059943, 1.667088, 1.439900, -1000, -1.164922, 0.948034, 1.292717});

  EXPECT_FALSE(franka->in_range(joint_positions));
  EXPECT_FALSE(franka->in_range(joint_state));

  joint_velocities.set_velocities(std::vector<double>{-0.059943, 31.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  joint_torques.set_torques(std::vector<double>{-0.329909, -0.235174, -1.881858, -922.491807, 0.674615, 0.996670, 0.345810});

  EXPECT_FALSE(franka->in_range(joint_velocities));
  EXPECT_FALSE(franka->in_range(joint_torques));
}

TEST_F(RobotModelKinematicsTest, TestClamp) {
  state_representation::JointPositions joint_positions("robot", franka->get_joint_frames());
  state_representation::JointVelocities joint_velocities("robot", franka->get_joint_frames());
  state_representation::JointTorques joint_torques("robot", franka->get_joint_frames());
  state_representation::JointState joint_state("robot", franka->get_joint_frames());

  joint_positions.set_positions(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 1000});
  joint_velocities.set_velocities(std::vector<double>{-0.059943, 31.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  joint_torques.set_torques(std::vector<double>{-0.329909, -0.235174, -1.881858, -922.491807, 0.674615, 0.996670, 0.345810});

  joint_state.set_positions(std::vector<double>{-0.059943, 1.667088, 1.439900, -1000, -1.164922, 0.948034, 1.292717});
  joint_state.set_velocities(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  joint_state.set_torques(std::vector<double>{-0.329909, -0.235174, -1.881858, -2.491807, 0.674615, 0.996670, 0.345810});

  EXPECT_FALSE(franka->in_range(joint_positions));
  EXPECT_FALSE(franka->in_range(joint_velocities));
  EXPECT_FALSE(franka->in_range(joint_torques));
  EXPECT_FALSE(franka->in_range(joint_state));

  EXPECT_TRUE(franka->in_range(franka->clamp_in_range(joint_positions)));
  EXPECT_TRUE(franka->in_range(franka->clamp_in_range(joint_velocities)));
  EXPECT_TRUE(franka->in_range(franka->clamp_in_range(joint_torques)));
  EXPECT_TRUE(franka->in_range(franka->clamp_in_range(joint_state)));
}

TEST_F(RobotModelKinematicsTest, TestInverseKinematics) {
  state_representation::JointState config1("robot", franka->get_joint_frames());
  state_representation::JointState config2("robot", franka->get_joint_frames());
  state_representation::JointState config3("robot", franka->get_joint_frames());
  // Random test configurations
  config1.set_positions(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  config2.set_positions(std::vector<double>{2.648782, -0.553976, 0.801067, -2.042097, -1.642935, 2.946476, 1.292717});
  config3.set_positions(std::vector<double>{-0.329909, -0.235174, -1.881858, -2.491807, 0.674615, 0.996670, 0.345810});

  std::vector<state_representation::JointState> test_configs = {config1, config2, config3};
  double tol = 1e-3;
  std::chrono::nanoseconds dt(static_cast<int>(1e9));
  InverseKinematicsParameters param = InverseKinematicsParameters();
  param.tolerance = tol;

  for (auto& config : test_configs) {
    state_representation::CartesianPose reference = franka->forward_kinematics(config, "panda_link8");
    state_representation::JointPositions q = franka->inverse_kinematics(reference, param, "panda_link8");
    state_representation::CartesianPose X = franka->forward_kinematics(q, "panda_link8");
    EXPECT_TRUE(((reference - X) / dt).data().cwiseAbs().maxCoeff() < tol);
  }
}

TEST_F(RobotModelKinematicsTest, TestInverseKinematicsIKDoesNotConverge) {
  state_representation::JointState config("robot", franka->get_joint_frames());
  // Random test configuration
  config.set_positions(std::vector<double>{-0.059943, 1.667088, 1.439900, -1.367141, -1.164922, 0.948034, 2.239983});
  InverseKinematicsParameters param = InverseKinematicsParameters();
  param.max_number_of_iterations = 1;

  state_representation::CartesianPose reference = franka->forward_kinematics(config, "panda_link8");
  EXPECT_THROW(franka->inverse_kinematics(reference, param, "panda_link8"),
               exceptions::InverseKinematicsNotConvergingException);
}

TEST_F(RobotModelKinematicsTest, ComputeJacobian) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::Jacobian jac = franka->compute_jacobian(test_configs[config]);

    for (Eigen::MatrixXd::Index element = 0; element < jac.data().size(); ++element) {
      EXPECT_NEAR(jac.data()(element), test_jacobian_ee_expects[config](element), tol);
    }
  }
}

TEST_F(RobotModelKinematicsTest, ComputeJacobianTimeDerivative) {
  for (std::size_t config = 0; config < test_configs.size(); ++config) {
    state_representation::JointVelocities velocities = test_configs[config];
    state_representation::JointPositions pos1 = test_configs[config];
    state_representation::JointPositions pos2 = pos1 + std::chrono::milliseconds(1) * velocities;
    auto jac1 = franka->compute_jacobian(pos1);
    auto jac2 = franka->compute_jacobian(pos2);

    auto jac1_dt = franka->compute_jacobian_time_derivative(pos1, velocities);
    auto jac2_expect = jac1.data() + 0.001 * jac1_dt;

    for (Eigen::Index index = 0; index < jac1_dt.size(); ++index) {
      EXPECT_NEAR(jac2.data()(index), jac2_expect(index), 1e-3);
    }

    velocities.set_zero();
    auto jt = franka->compute_jacobian_time_derivative(pos1, velocities);
    EXPECT_NEAR(jt.sum(), 0, tol);
  }
}
