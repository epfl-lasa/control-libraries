#pragma once

#include <string>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

namespace robot_model {
/**
 * @brief parameters for the inverse geometry function
 * @param damp damping added to the diagonal of J*Jt in order to avoid the singularity
 * @param alpha alpha €]0,1], it is used to make Newthon-Raphson method less aggressive
 * @param gamma gamma €]0,1], represents the strength of the repulsive potential field in the Clamping Weighted Least-Norm method
 * @param margin the distance from the joint limit on which we want to penalize the joint position
 * @param tolerance the maximum error tolerated between the desired cartesian state and the one obtained by the returned joint positions
 * @param max_number_of_iterations the maximum number of iterations that the algorithm do for solving the inverse geometry
 */
struct InverseGeometryParameters {
  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int max_number_of_iterations = 1000;
};

/**
 * @brief parameters for the inverse kinematics function
 * @param alpha for the time optimization
 * @param epsilon minimal time for the time optimization
 * @param proportional_gain gain to weight the cartesian coordinates in the gradient
 * @param linear_velocity_limit maximum linear velocity allowed in Cartesian space (m/s)
 * @param angular_velocity_limit maximum angular velocity allowed in Cartesian space (rad/s)
 */
struct InverseKinematicsParameters {
  double alpha = 0.1;
  double epsilon = 1e-2;
  double proportional_gain = 1.0;
  double linear_velocity_limit = 2.0;
  double angular_velocity_limit = 2.0;
};

/**
 * @class Model
 * @brief The Model class is a wrapper around pinocchio dynamic computation library with state_representation
 * encapsulations.
 */
class Model {
private:
  // @format:off
  std::shared_ptr<state_representation::Parameter<std::string>> robot_name_;///< name of the robot
  std::shared_ptr<state_representation::Parameter<std::string>> urdf_path_; ///< path to the urdf file
  std::vector<std::string> frame_names_;                                    ///< name of the frames
  pinocchio::Model robot_model_;                                            ///< the robot model with pinocchio
  pinocchio::Data robot_data_;                                              ///< the robot data with pinocchio
  OsqpEigen::Solver solver_;                                                ///< osqp solver for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> hessian_;                                     ///< hessian matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd gradient_;                                                ///< gradient vector for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> constraint_matrix_;                           ///< constraint matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd lower_bound_constraints_;                                 ///< lower bound matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd upper_bound_constraints_;                                 ///< upper bound matrix for the quadratic programming based inverse kinematics
  // @format:on
  /**
   * @brief Initialize the pinocchio model from the URDF
   */
  void init_model();

  /**
   * @brief initialize the constraints for the QP solver
   * @param parameters the parameters of the inverse kinematics algorithm
   */
  bool init_qp_solver(const InverseKinematicsParameters& parameters);

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

  /**
   * @brief Check if the vector's elements are inside the parameter limits
   * @param vector the vector to check
   * @param lower_limits the lower bounds of the limits
   * @param upper_limits the upper bounds of the limits
   * @return true if all the elements are inside at their limits, false otherwise.
   */
  static bool in_range(const Eigen::VectorXd& vector,
                       const Eigen::VectorXd& lower_limits,
                       const Eigen::VectorXd& upper_limits);

  /**
   * @brief Clamp the vector's elements according to the parameter limits
   * @param vector the vector to clamp
   * @param lower_limits the lower bounds of the limits
   * @param upper_limits the upper bounds of the limits
   * @return the clamped vector
   */
  static Eigen::VectorXd clamp_in_range(const Eigen::VectorXd& vector,
                                        const Eigen::VectorXd& lower_limits,
                                        const Eigen::VectorXd& upper_limits);

  /**
   * @brief Compute the weighted matrix of the algorithm "Clamping Weighted Least-Norm"
   * @param joint_positions the joint position at the current iteration in the inverse geometry problem
   * @param margin the distance from the joint limit on which we want to penalize the joint position
   * @return the weighted matrix
   */
  Eigen::MatrixXd cwln_weighted_matrix(const state_representation::JointPositions& joint_positions, double margin);

  /**
   * @brief Compute the repulsive potential field of the algorithm "Clamping Weighted Least-Norm"
   * @param joint_positions the joint position at the current iteration in the inverse geometry problem
   * @param margin the distance from the joint limit on which we want to penalize the joint position
   * @return the repulsive potential field
   */
  Eigen::VectorXd cwln_repulsive_potential_field(const state_representation::JointPositions& joint_positions,
                                                 double margin);

public:
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
   * @brief Swap the values of the two Model
   * @param model1 Model to be swapped with 2
   * @param model2 Model to be swapped with 1
   */
  friend void swap(Model& model1, Model& model2);

  /**
   * @brief Copy assignment operator that have to be defined due to the custom assignment operator
   * @param model the model with value to assign
   * @return reference to the current model with new values
   */
  Model& operator=(const Model& Model);

  /**
   * @brief Creates a URDF file with desired path and name from a string (possibly the robot description
   * string from the ROS parameter server)
   * @param urdf_string string containing the URDF description of the robot
   * @param desired_path desired path and name of the created URDF file as string
   * @return bool if operation was successful
   */
  static bool create_urdf_from_string(const std::string& urdf_string, const std::string& desired_path);

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
   * @brief Compute the inverse geometry, i.e. joint values from the pose of the end-effector in a iteratively manner
   * @param desired_cartesian_state containing the desired pose of the end-effector
   * @param frame_name name of the frame at which we want to extract the pose
   * @param parameters parameters of the inverse geometry algorithm (default is default values of the
   * InverseGeometryParameters structure)
   * @return the joint positions of the robot
   */
  state_representation::JointPositions inverse_geometry(const state_representation::CartesianState& desired_cartesian_state,
                                                        const std::string& frame_name = "",
                                                        const InverseGeometryParameters& parameters = InverseGeometryParameters());

  /**
   * @brief Compute the inverse geometry, i.e. joint values from the pose of the end-effector
   * @param desired_cartesian_state containing the desired pose of the end-effector
   * @param current_joint_state current state of the robot containing the generalized position
   * @param frame_name name of the frame at which we want to extract the pose
   * @param parameters parameters of the inverse geometry algorithm (default is default values of the
   * InverseGeometryParameters structure)
   * @return the joint positions of the robot
   */
  state_representation::JointPositions inverse_geometry(const state_representation::CartesianState& desired_cartesian_state,
                                                        const state_representation::JointState& current_joint_state,
                                                        const std::string& frame_name = "",
                                                        const InverseGeometryParameters& parameters = InverseGeometryParameters());

  /**
   * @brief Compute the forward kinematics, i.e. the twist of the end-effector from the joint velocities
   * @param joint_state the joint state of the robot
   * @return the twist of the end-effector
   */
  state_representation::CartesianTwist forward_kinematics(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the inverse kinematic, i.e. joint velocities from the velocities of the frames in parameter
   * @param joint_state usually the current joint state, used to compute the jacobian matrix
   * @param cartesian_twist vector of twist
   * @param parameters parameters of the inverse kinematics algorithm (default is default values of the
   * InverseKinematicsParameters structure)
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_kinematics(const state_representation::JointState& joint_state,
                                                           const std::vector<state_representation::CartesianTwist>& cartesian_states,
                                                           const InverseKinematicsParameters& parameters = InverseKinematicsParameters());

  /**
   * @brief Compute the inverse kinematic, i.e. joint velocities from the twist of the end-effector
   * @param joint_state usually the current joint state, used to compute the jacobian matrix
   * @param cartesian_twist containing the twist of the end-effector
   * @param parameters parameters of the inverse kinematics algorithm (default is default values of the
   * InverseKinematicsParameters structure)
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_kinematics(const state_representation::JointState& joint_state,
                                                           const state_representation::CartesianTwist& cartesian_state,
                                                           const InverseKinematicsParameters& parameters = InverseKinematicsParameters());

  /**
   * @brief Helper function to print the qp_problem (for debugging)
   */
  void print_qp_problem();

  /**
   * @brief Check if the joint positions are inside the limits provided by the model
   * @param joint_positions the joint positions to check
   * @return true if the positions are inside their limits, false otherwise.
   */
  bool in_range(const state_representation::JointPositions& joint_positions) const;

  /**
   * @brief Check if the joint velocities are inside the limits provided by the model
   * @param joint_velocities the joint velocities to check
   * @return true if the velocities are inside their limits, false otherwise.
   */
  bool in_range(const state_representation::JointVelocities& joint_velocities) const;

  /**
   * @brief Check if the joint torques are inside the limits provided by the model
   * @param joint_torques the joint torques to check
   * @return true if the torques are inside their limits, false otherwise.
   */
  bool in_range(const state_representation::JointTorques& joint_torques) const;

  /**
   * @brief Check if the joint state variables (positions, velocities & torques) are inside the limits provided by
   * the model
   * @param joint_state the joint state to check
   * @return true if the state variables are inside the limits, false otherwise.
   */
  bool in_range(const state_representation::JointState& joint_state) const;

  /**
   * @brief Clamp the joint state variables (positions, velocities & torques) according to the limits provided by
   * the model
   * @param joint_state the joint state to be clamped
   * @return the clamped joint states
   */
  state_representation::JointState clamp_in_range(const state_representation::JointState& joint_state) const;
};

inline void swap(Model& model1, Model& model2) {
  std::swap(model1.robot_name_, model2.robot_name_);
  std::swap(model1.urdf_path_, model2.urdf_path_);
  // initialize both models
  model1.init_model();
  model2.init_model();
}

inline Model& Model::operator=(const Model& model) {
  Model tmp(model);
  swap(*this, tmp);
  return *this;
}


inline const std::string& Model::get_robot_name() const {
  return this->robot_name_->get_value();
}

inline void Model::set_robot_name(const std::string& robot_name) {
  this->robot_name_->set_value(robot_name);
}

inline const std::string& Model::get_urdf_path() const {
  return this->urdf_path_->get_value();
}

inline unsigned int Model::get_number_of_joints() const {
  return this->robot_model_.nq;
}

inline std::vector<std::string> Model::get_joint_frames() const {
  // model contains a first joint called universe that needs to be discarded
  std::vector<std::string> joint_frames(this->robot_model_.names.begin() + 1, this->robot_model_.names.end());
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
}// namespace robot_model