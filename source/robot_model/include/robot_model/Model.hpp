#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <state_representation/Parameters/Parameter.hpp>
#include <state_representation/Parameters/ParameterInterface.hpp>
#include <state_representation/Robot/Jacobian.hpp>
#include <state_representation/Robot/JointState.hpp>
#include <state_representation/Space/Cartesian/CartesianState.hpp>

namespace RobotModel {
class Model {
private:
  // @format:off
  std::shared_ptr<StateRepresentation::Parameter<std::string>> robot_name_;       ///< name of the robot
  std::shared_ptr<StateRepresentation::Parameter<std::string>> urdf_path_;        ///< path to the urdf file
  pinocchio::Model robot_model_;                                                  ///< the robot model with pinocchio
  pinocchio::Data robot_data_;                                                    ///< the robot data with pinocchio
  OsqpEigen::Solver solver_;                                                      ///< osqp solver for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> hessian_;                                           ///< hessian matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd gradient_;                                                      ///< gradient vector for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> constraint_matrix_;                                 ///< constraint matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd lower_bound_constraints_;                                       ///< lower bound matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd upper_bound_constraints_;                                       ///< upper bound matrix for the quadratic programming based inverse kinematics
  std::shared_ptr<StateRepresentation::Parameter<double>> alpha_;                 ///< gain for the time optimization in the quadratic programming based inverse kinematics
  std::shared_ptr<StateRepresentation::Parameter<double>> epsilon_;               ///< minimal time for the time optimization in the quadratic programming based inverse kinematics
  std::shared_ptr<StateRepresentation::Parameter<double>> linear_velocity_limit_; ///< maximum linear velocity allowed in the inverse kinematics
  std::shared_ptr<StateRepresentation::Parameter<double>> angular_velocity_limit_;///< maximum angular velocity allowed in the inverse kinematics
  std::shared_ptr<StateRepresentation::Parameter<double>> proportional_gain_;     ///< gain to weight the cartesian coordinates in the gradient
  // @format:on
  /**
   * @brief initialize the constraints for the QP solver
   */
  bool init_qp_solver();

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
   * @brief Desctructor
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
   * @brief Getter of the joint names attribute
   * @return the vector of joint names
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Getter of the number of joints
   * @return the number of joints
   */
  unsigned int get_nb_joints() const;

  /**
   * @brief Initialize the pinocchio model from the URDF
   */
  void init_model();

  /**
   * @brief Compute the jacobian from a given joint state at the frame in parameter
   * @param joint_state containing the joint values of the robot
   * @param joint_id id of the frame at which to compute the jacobian
   * @return the jacobian matrix
   */
  StateRepresentation::Jacobian compute_jacobian(const StateRepresentation::JointState& joint_state, int frame_id);

  /**
   * @brief Compute the jacobian from a given joint state at the frame given in parameter
   * @param joint_state containing the joint values of the robot
   * @param frame_name name of the frame at which to compute the jacobian, if empty computed for the last frame
   * @return the jacobian matrix
   */
  StateRepresentation::Jacobian compute_jacobian(const StateRepresentation::JointState& joint_state,
                                                 const std::string& frame_name = "");

  /**
   * @brief Compute the forward geometry, i.e. the pose of certain frames from the joint values
   * @param joint_state the joint state of the robot
   * @param frame_ids ids of the frames at which we want to extract the pose
   * @return the poses of the desired poses
   */
  std::vector<StateRepresentation::CartesianPose> forward_geometry(const StateRepresentation::JointState& joint_state,
                                                                   const std::vector<unsigned int>& frame_ids);

  /**
   * @brief Compute the forward geometry, i.e. the pose of certain frames from the joint values
   * @param joint_state the joint state of the robot
   * @param frame_names names of the frames at which we want to extract the pose
   * @return the pose of desired frames
   */
  std::vector<StateRepresentation::CartesianPose> forward_geometry(const StateRepresentation::JointState& joint_state,
                                                                   const std::vector<std::string>& frame_names);

  /**
 * @brief Compute the forward geometry, i.e. the pose of the frame from the joint values
 * @param joint_state the joint state of the robot
 * @param frame_name name of the frame at which we want to extract the pose
 * @return the pose of the desired frame
 */
  StateRepresentation::CartesianPose forward_geometry(const StateRepresentation::JointState& joint_state,
                                                      const std::string& frame_names);

  /**
   * @brief Compute the inverse geometry, i.e. joint values from the pose of the end-effector
   * @param cartesian_state containing the pose of the end-effector
   * @return the joint state of the robot
   */
  const StateRepresentation::JointPositions inverse_geometry(const StateRepresentation::CartesianState& cartesian_state) const;

  /**
   * @brief Compute the forward kinematic, i.e. the twist of the end-effector from the joint velocities
   * @param joint_state the joint state of the robot
   * @return the twist of the end-effector
   */
  const StateRepresentation::CartesianTwist forward_kinematic(const StateRepresentation::JointState& joint_state);

  /**
   * @brief Compute the inverse kinematics, i.e. joint velocities from the velocities of the frames in parameter
   * @param joint_state usually the current joint state, used to compute the jacobian matrix
   * @param cartesian_states vector of twist
   * @return the joint velocities of the robot
   */
  StateRepresentation::JointVelocities inverse_kinematic(const StateRepresentation::JointState& joint_state,
                                                         const std::vector<StateRepresentation::CartesianState>& cartesian_states);

  /**
   * @brief Compute the inverse kinematic, i.e. joint velocities from the twist of the end-effector
   * @param joint_state usually the current joint state, used to compute the jacobian matrix
   * @param cartesian_state containing the twist of the end-effector
   * @return the joint velocities of the robot
   */
  StateRepresentation::JointVelocities inverse_kinematic(const StateRepresentation::JointState& joint_state,
                                                         const StateRepresentation::CartesianState& cartesian_state);

  /**
   * @brief Helper funtion to print the qp_problem (for debugging)
   */
  void print_qp_problem();

  /**
   * @brief Return a list of all the parameters of the model
   * @return the list of parameters
   */
  const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
};

inline const std::string& Model::get_robot_name() const {
  return this->robot_name_->get_value();
}

inline const std::string& Model::get_urdf_path() const {
  return this->urdf_path_->get_value();
}

inline void Model::set_urdf_path(const std::string& urdf_path) {
  this->urdf_path_->set_value(urdf_path);
}

inline const std::vector<std::string>& Model::get_joint_names() const {
  return this->joint_names_;
}

inline unsigned int Model::get_nb_joints() const {
  // this is technically not correct
  return this->robot_model_.nv;
}

inline void Model::init_model() {
  pinocchio::urdf::buildModel(this->get_urdf_path(), this->robot_model_);
  this->robot_data_ = pinocchio::Data(this->robot_model_);
}

inline const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Model::get_parameters() const {
  std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
  param_list.push_back(this->alpha_);
  param_list.push_back(this->epsilon_);
  param_list.push_back(this->linear_velocity_limit_);
  param_list.push_back(this->angular_velocity_limit_);
  param_list.push_back(this->proportional_gain_);
  return param_list;
}
}