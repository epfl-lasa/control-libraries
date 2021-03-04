#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/space/Cartesian/CartesianState.hpp>

namespace RobotModel {
class Model {
private:
  // @format:off
  std::shared_ptr<state_representation::Parameter<std::string>> robot_name_;       ///< name of the robot
  std::shared_ptr<state_representation::Parameter<std::string>> urdf_path_;        ///< path to the urdf file
  std::vector<std::string> frame_names_;                                          ///< name of the frames
  pinocchio::Model robot_model_;                                                  ///< the robot model with pinocchio
  pinocchio::Data robot_data_;                                                    ///< the robot data with pinocchio
  OsqpEigen::Solver solver_;                                                      ///< osqp solver for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> hessian_;                                           ///< hessian matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd gradient_;                                                      ///< gradient vector for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> constraint_matrix_;                                 ///< constraint matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd lower_bound_constraints_;                                       ///< lower bound matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd upper_bound_constraints_;                                       ///< upper bound matrix for the quadratic programming based inverse kinematics
  std::shared_ptr<state_representation::Parameter<double>> alpha_;                 ///< gain for the time optimization in the quadratic programming based inverse kinematics
  std::shared_ptr<state_representation::Parameter<double>> epsilon_;               ///< minimal time for the time optimization in the quadratic programming based inverse kinematics
  std::shared_ptr<state_representation::Parameter<double>> linear_velocity_limit_; ///< maximum linear velocity allowed in the inverse kinematics
  std::shared_ptr<state_representation::Parameter<double>> angular_velocity_limit_;///< maximum angular velocity allowed in the inverse kinematics
  std::shared_ptr<state_representation::Parameter<double>> proportional_gain_;     ///< gain to weight the cartesian coordinates in the gradient
  // @format:on
  /**
   * @brief initialize the constraints for the QP solver
   */
  bool init_qp_solver();

  /**
   * @brief Compute the jacobian from a given joint state at the frame in parameter
   * @param joint_state containing the joint values of the robot
   * @param joint_id id of the frame at which to compute the jacobian
   * @return the jacobian matrix
   */
  state_representation::Jacobian compute_jacobian(const state_representation::JointState& joint_state,
                                                  unsigned int frame_id);

  /**
   * @brief Compute the forward geometry, i.e. the pose of certain frames from the joint values
   * @param joint_state the joint state of the robot
   * @param frame_ids ids of the frames at which we want to extract the pose
   * @return the desired poses
   */
  std::vector<state_representation::CartesianPose> forward_geometry(const state_representation::JointState& joint_state,
                                                                    const std::vector<unsigned int>& frame_ids);

  /**
   * @brief Compute the forward geometry, i.e. the pose of certain frames from the joint values for a single frame
   * @param joint_state the joint state of the robot
   * @param frame_id id of the frames at which we want to extract the pose
   * @return the desired pose
   */
  state_representation::CartesianPose forward_geometry(const state_representation::JointState& joint_state,
                                                       unsigned int frame_id);

public:
  /**
   * @brief Empty constructor
   */
  explicit Model();

  /**
   * @brief Constructor with robot name and path to URDF file
   */
  explicit Model(const std::string& robot_name, const std::string& urdf_path);

  /**
   * @brief Copy constructor
   * @param model the model to copy
   */
  Model(const Model& model);

  /**
   * @brief Destructor
   */
  ~Model();

  /**
   * @brief Copy assignment operator that have to be defined due to the custom assignment operator
   * @param model the model with value to assign
   * @return reference to the current model with new values
   */
  Model& operator=(const Model& Model);

  /**
   * @brief Getter of the robot name
   * @return the robot name
   */
  const std::string& get_robot_name() const;

  /**
   * @brief Setter of the robot name
   * @param robot_name the new value of the robot name
   */
  void set_robot_name(const std::string& robot_name);

  /**
   * @brief Getter of the URDF path
   * @return the URDF path
   */
  const std::string& get_urdf_path() const;

  /**
   * @brief Setter of the URDF path
   * @param urdf_path the new value of the path
   */
  void set_urdf_path(const std::string& urdf_path);

  /**
   * @brief Getter of the number of joints
   * @return the number of joints
   */
  unsigned int get_number_of_joints() const;

  /**
   * Getter of the joint frames from the model
   * @return the joint frames
   */
  std::vector<std::string> get_joint_frames() const;

  /**
   * Getter of the frames from the model
   * @return the frame names
   */
  std::vector<std::string> get_frames() const;

  /**
   * @brief Getter of the gravity vector
   * @return Eigen::Vector3d the gravity vector
   */
  Eigen::Vector3d get_gravity_vector() const;

  /**
   * @brief Setter of the gravity vector
   * @param gravity the gravity vector
   */
  void set_gravity_vector(const Eigen::Vector3d& gravity);

  /**
   * @brief Initialize the pinocchio model from the URDF
   */
  void init_model();

  /**
   * @brief Compute the jacobian from a given joint state at the frame given in parameter
   * @param joint_state containing the joint values of the robot
   * @param frame_name name of the frame at which to compute the jacobian, if empty computed for the last frame
   * @return the jacobian matrix
   */
  state_representation::Jacobian compute_jacobian(const state_representation::JointState& joint_state,
                                                  const std::string& frame_name = "");

  /**
   * @brief Compute the Inertia matrix from a given joint positions
   * @param joint_positions containing the joint positions values of the robot
   * @return the inertia matrix
   */
  Eigen::MatrixXd compute_inertia_matrix(const state_representation::JointPositions& joint_positions);

  /**
   * @brief Compute the Inertia torques, i.e the inertia matrix multiplied by the joint accelerations. Joint positions
   * are needed as well for computations of the inertia matrix
   * @param joint_state containing the joint positions and accelerations values of the robot
   * @return the inertia torques as a JointTorques
   */
  state_representation::JointTorques compute_inertia_torques(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the Coriolis matrix from a given joint state
   * @param joint_state containing the joint positions & velocities values of the robot
   * @return the Coriolis matrix
   */
  Eigen::MatrixXd compute_coriolis_matrix(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the Coriolis torques, i.e. the Coriolis matrix multiplied by the joint velocities and express the
   * result as a JointTorques
   * @param joint_state containing the joint positions & velocities values of the robot
   * @return the Coriolis torques as a JointTorques
   */
  state_representation::JointTorques compute_coriolis_torques(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the gravity torques
   * @param joint_positions containing the joint positions of the robot
   * @return the gravity torque as a JointTorques
   */
  state_representation::JointTorques compute_gravity_torques(const state_representation::JointPositions& joint_positions);

  /**
   * @brief Compute the forward geometry, i.e. the pose of certain frames from the joint values
   * @param joint_state the joint state of the robot
   * @param frame_names names of the frames at which we want to extract the pose
   * @return the pose of desired frames
   */
  std::vector<state_representation::CartesianPose> forward_geometry(const state_representation::JointState& joint_state,
                                                                    const std::vector<std::string>& frame_names);

  /**
 * @brief Compute the forward geometry, i.e. the pose of the frame from the joint values
 * @param joint_state the joint state of the robot
 * @param frame_name name of the frame at which we want to extract the pose
 * @return the pose of the desired frame
 */
  state_representation::CartesianPose forward_geometry(const state_representation::JointState& joint_state,
                                                       std::string frame_name = "");

  /**
   * @brief Compute the inverse geometry, i.e. joint values from the pose of the end-effector
   * @param cartesian_state containing the pose of the end-effector
   * @return the joint state of the robot
   */
  state_representation::JointPositions inverse_geometry(const state_representation::CartesianState& cartesian_state) const;

  /**
   * @brief Compute the forward kinematic, i.e. the twist of the end-effector from the joint velocities
   * @param joint_state the joint state of the robot
   * @return the twist of the end-effector
   */
  state_representation::CartesianTwist forward_kinematic(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the inverse kinematic, i.e. joint velocities from the velocities of the frames in parameter
   * @param joint_state usually the current joint state, used to compute the jacobian matrix
   * @param cartesian_states vector of twist
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_kinematic(const state_representation::JointState& joint_state,
                                                          const std::vector<state_representation::CartesianState>& cartesian_states);

  /**
   * @brief Compute the inverse kinematic, i.e. joint velocities from the twist of the end-effector
   * @param joint_state usually the current joint state, used to compute the jacobian matrix
   * @param cartesian_state containing the twist of the end-effector
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_kinematic(const state_representation::JointState& joint_state,
                                                          const state_representation::CartesianState& cartesian_state);

  /**
   * @brief Helper function to print the qp_problem (for debugging)
   */
  void print_qp_problem();

  /**
   * @brief Return a list of all the parameters of the model
   * @return the list of parameters
   */
  std::list<std::shared_ptr<state_representation::ParameterInterface>> get_parameters() const;
};

inline const std::string& Model::get_robot_name() const {
  return this->robot_name_->get_value();
}

inline void Model::set_robot_name(const std::string& robot_name) {
  this->robot_name_->set_value(robot_name);
}

inline const std::string& Model::get_urdf_path() const {
  return this->urdf_path_->get_value();
}

inline void Model::set_urdf_path(const std::string& urdf_path) {
  this->urdf_path_->set_value(urdf_path);
}

inline unsigned int Model::get_number_of_joints() const {
  return this->robot_model_.nq;
}

inline std::vector<std::string> Model::get_joint_frames() const {
  // model contains a first joint called universe that needs to be discarded
  std::vector<std::string> joint_frames(this->robot_model_.names.begin() + 1,
                                       this->robot_model_.names.end());
  return joint_frames;
}

inline std::vector<std::string> Model::get_frames() const {
  return this->frame_names_;
}

inline Eigen::Vector3d Model::get_gravity_vector() const {
  return this->robot_model_.gravity.linear();
}

inline void Model::set_gravity_vector(const Eigen::Vector3d& gravity) {
  this->robot_model_.gravity.linear(gravity);
}

inline state_representation::CartesianPose Model::forward_geometry(const state_representation::JointState& joint_state,
                                                                  unsigned int frame_id) {
  return this->forward_geometry(joint_state, std::vector<unsigned int>{frame_id}).front();
}

inline state_representation::CartesianPose Model::forward_geometry(const state_representation::JointState& joint_state,
                                                                  std::string frame_name) {
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_name = this->robot_model_.frames.back().name;
  }
  return this->forward_geometry(joint_state, std::vector<std::string>{frame_name}).front();
}

inline std::list<std::shared_ptr<state_representation::ParameterInterface>> Model::get_parameters() const {
  std::list<std::shared_ptr<state_representation::ParameterInterface>> param_list;
  param_list.push_back(this->alpha_);
  param_list.push_back(this->epsilon_);
  param_list.push_back(this->linear_velocity_limit_);
  param_list.push_back(this->angular_velocity_limit_);
  param_list.push_back(this->proportional_gain_);
  return param_list;
}
}// namespace RobotModel