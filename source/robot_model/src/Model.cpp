#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "robot_model/Model.hpp"
#include "robot_model/exceptions/FrameNotFoundException.hpp"
#include "robot_model/exceptions/InverseKinematicsNotConvergingException.hpp"
#include "robot_model/exceptions/InvalidJointStateSizeException.hpp"

namespace robot_model {
Model::Model(const std::string& robot_name, const std::string& urdf_path) :
    robot_name_(std::make_shared<state_representation::Parameter<std::string>>("robot_name", robot_name)),
    urdf_path_(std::make_shared<state_representation::Parameter<std::string>>("urdf_path", urdf_path)) {
  this->init_model();
}

Model::Model(const Model& model) :
    robot_name_(model.robot_name_),
    urdf_path_(model.urdf_path_) {
  this->init_model();
}

bool Model::create_urdf_from_string(const std::string& urdf_string, const std::string& desired_path) {
  std::ofstream file(desired_path);
  if (file.good() && file.is_open()) {
    file << urdf_string;
    file.close();
    return true;
  }
  return false;
}

void Model::init_model() {
  pinocchio::urdf::buildModel(this->get_urdf_path(), this->robot_model_);
  this->robot_data_ = pinocchio::Data(this->robot_model_);
  // get the frame names
  std::vector<std::string> frames;
  for (auto& f : this->robot_model_.frames) {
    frames.push_back(f.name);
  }
  // remove universe and root_joint frame added by Pinocchio
  this->frame_names_ = std::vector<std::string>(frames.begin() + 2, frames.end());
}

bool Model::init_qp_solver(const InverseVelocityParameters& parameters) {
  // clear the solver
  this->solver_.data()->clearHessianMatrix();
  this->solver_.data()->clearLinearConstraintsMatrix();
  this->solver_.clearSolver();

  unsigned int nb_joints = this->get_number_of_joints();
  // initialize the matrices
  this->hessian_ = Eigen::SparseMatrix<double>(nb_joints + 1, nb_joints + 1);
  this->gradient_ = Eigen::VectorXd::Zero(nb_joints + 1);
  this->constraint_matrix_ = Eigen::SparseMatrix<double>(4 * nb_joints + 1 + 6, nb_joints + 1);
  this->lower_bound_constraints_ = Eigen::VectorXd::Zero(4 * nb_joints + 1 + 6);
  this->upper_bound_constraints_ = Eigen::VectorXd::Zero(4 * nb_joints + 1 + 6);

  // reserve the size of the matrices
  this->hessian_.reserve(nb_joints * nb_joints + 1);
  this->constraint_matrix_.reserve(6 * nb_joints + 2 * (nb_joints * nb_joints + nb_joints) + 4 * nb_joints + 7);

  Eigen::VectorXd lower_position_limit = this->robot_model_.lowerPositionLimit;
  Eigen::VectorXd upper_position_limit = this->robot_model_.upperPositionLimit;
  Eigen::VectorXd velocity_limit = this->robot_model_.velocityLimit;

  // configure the QP problem
  this->solver_.settings()->setVerbosity(false);
  this->solver_.settings()->setWarmStart(true);

  // joint dependent constraints
  for (unsigned int n = 0; n < nb_joints; ++n) {
    // joint limits
    this->constraint_matrix_.coeffRef(n, n) = 1.0;
    this->constraint_matrix_.coeffRef(n, nb_joints) = -upper_position_limit(n);// joint_limit max
    this->lower_bound_constraints_(n) = -std::numeric_limits<double>::infinity();
    this->constraint_matrix_.coeffRef(n + nb_joints, n) = 1.0;
    this->constraint_matrix_.coeffRef(n + nb_joints, nb_joints) = -lower_position_limit(n);// joint limit min
    this->upper_bound_constraints_(n + nb_joints) = std::numeric_limits<double>::infinity();
    // joint velocity limits
    this->constraint_matrix_.coeffRef(n + 2 * nb_joints, n) = 1.0;
    this->constraint_matrix_.coeffRef(n + 2 * nb_joints, nb_joints) = -velocity_limit(n);// joint_limit max
    this->lower_bound_constraints_(n + 2 * nb_joints) = -std::numeric_limits<double>::infinity();
    this->constraint_matrix_.coeffRef(n + 3 * nb_joints, n) = 1.0;
    this->constraint_matrix_.coeffRef(n + 3 * nb_joints, nb_joints) = velocity_limit(n);// joint limit min
    this->upper_bound_constraints_(n + 3 * nb_joints) = std::numeric_limits<double>::infinity();
  }

  // time constraint
  this->hessian_.coeffRef(nb_joints, nb_joints) = parameters.alpha;
  this->constraint_matrix_.coeffRef(4 * nb_joints, nb_joints) = 1.0;
  this->lower_bound_constraints_(4 * nb_joints) = parameters.epsilon;
  this->upper_bound_constraints_(4 * nb_joints) = std::numeric_limits<double>::infinity();
  // cartesian velocity constraints
  for (unsigned int i = 0; i < 3; ++i) {
    // linear velocity
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 1, nb_joints) = parameters.linear_velocity_limit;
    this->upper_bound_constraints_(4 * nb_joints + i + 1) = std::numeric_limits<double>::infinity();
    // angular velocity
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 4, nb_joints) = parameters.angular_velocity_limit;
    this->upper_bound_constraints_(4 * nb_joints + i + 4) = std::numeric_limits<double>::infinity();
  }

  // set the initial data of the QP solver_
  this->solver_.data()->setNumberOfVariables(static_cast<int>(nb_joints) + 1);
  this->solver_.data()->setNumberOfConstraints(this->lower_bound_constraints_.size());
  if (!this->solver_.data()->setHessianMatrix(this->hessian_)) { return false; }
  if (!this->solver_.data()->setGradient(this->gradient_)) { return false; }
  if (!this->solver_.data()->setLinearConstraintsMatrix(this->constraint_matrix_)) { return false; }
  if (!this->solver_.data()->setLowerBound(this->lower_bound_constraints_)) { return false; }
  if (!this->solver_.data()->setUpperBound(this->upper_bound_constraints_)) { return false; }
  // instantiate the solver_
  return this->solver_.initSolver();
}

state_representation::Jacobian Model::compute_jacobian(const state_representation::JointPositions& joint_positions,
                                                       unsigned int frame_id) {
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw (exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints()));
  }
  // compute the Jacobian from the joint state
  pinocchio::Data::Matrix6x J(6, this->get_number_of_joints());
  J.setZero();
  pinocchio::computeFrameJacobian(this->robot_model_,
                                  this->robot_data_,
                                  joint_positions.data(),
                                  frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED,
                                  J);
  // the model does not have any reference frame
  return state_representation::Jacobian(this->get_robot_name(),
                                        this->get_joint_frames(),
                                        this->robot_model_.frames[frame_id].name,
                                        J,
                                        this->get_base_frame());
}

state_representation::Jacobian Model::compute_jacobian(const state_representation::JointPositions& joint_positions,
                                                       const std::string& frame_name) {
  unsigned int frame_id;
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_id = this->robot_model_.getFrameId(this->robot_model_.frames.back().name);
  } else {
    // throw error if specified frame does not exist
    if (!this->robot_model_.existFrame(frame_name)) {
      throw (exceptions::FrameNotFoundException(frame_name));
    }
    frame_id = this->robot_model_.getFrameId(frame_name);
  }
  return this->compute_jacobian(joint_positions, frame_id);
}

Eigen::MatrixXd Model::compute_jacobian_time_derivative(const state_representation::JointPositions& joint_positions,
                                                        const state_representation::JointVelocities& joint_velocities,
                                                        unsigned int frame_id) {
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw (exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints()));
  }
  if (joint_velocities.get_size() != this->get_number_of_joints()) {
    throw (exceptions::InvalidJointStateSizeException(joint_velocities.get_size(), this->get_number_of_joints()));
  }
  // compute the Jacobian from the joint state
  pinocchio::Data::Matrix6x dJ = Eigen::MatrixXd::Zero(6, this->get_number_of_joints());
  pinocchio::computeJointJacobiansTimeVariation(this->robot_model_,
                                                this->robot_data_,
                                                joint_positions.data(),
                                                joint_velocities.data());
  pinocchio::getFrameJacobianTimeVariation(this->robot_model_,
                                           this->robot_data_,
                                           frame_id,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ);
  // the model does not have any reference frame
  return dJ;
}

Eigen::MatrixXd Model::compute_jacobian_time_derivative(const state_representation::JointPositions& joint_positions,
                                                        const state_representation::JointVelocities& joint_velocities,
                                                        const std::string& frame_name) {
  unsigned int frame_id;
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_id = this->robot_model_.getFrameId(this->robot_model_.frames.back().name);
  } else {
    // throw error if specified frame does not exist
    if (!this->robot_model_.existFrame(frame_name)) {
      throw (exceptions::FrameNotFoundException(frame_name));
    }
    frame_id = this->robot_model_.getFrameId(frame_name);
  }
  return this->compute_jacobian_time_derivative(joint_positions, joint_velocities, frame_id);
}

Eigen::MatrixXd Model::compute_inertia_matrix(const state_representation::JointPositions& joint_positions) {
  // compute only the upper part of the triangular inertia matrix stored in robot_data_.M
  pinocchio::crba(this->robot_model_, this->robot_data_, joint_positions.data());
  // copy the symmetric lower part
  this->robot_data_.M.triangularView<Eigen::StrictlyLower>() =
      this->robot_data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return this->robot_data_.M;
}

state_representation::JointTorques Model::compute_inertia_torques(const state_representation::JointState& joint_state) {
  Eigen::MatrixXd inertia = this->compute_inertia_matrix(joint_state);
  return state_representation::JointTorques(joint_state.get_name(),
                                            joint_state.get_names(),
                                            inertia * joint_state.get_accelerations());
}

Eigen::MatrixXd Model::compute_coriolis_matrix(const state_representation::JointState& joint_state) {
  return pinocchio::computeCoriolisMatrix(this->robot_model_,
                                          this->robot_data_,
                                          joint_state.get_positions(),
                                          joint_state.get_velocities());
}

state_representation::JointTorques
Model::compute_coriolis_torques(const state_representation::JointState& joint_state) {
  Eigen::MatrixXd coriolis_matrix = this->compute_coriolis_matrix(joint_state);
  return state_representation::JointTorques(joint_state.get_name(),
                                            joint_state.get_names(),
                                            coriolis_matrix * joint_state.get_velocities());
}

state_representation::JointTorques
Model::compute_gravity_torques(const state_representation::JointPositions& joint_positions) {
  Eigen::VectorXd gravity_torque =
      pinocchio::computeGeneralizedGravity(this->robot_model_, this->robot_data_, joint_positions.data());
  return state_representation::JointTorques(joint_positions.get_name(), joint_positions.get_names(), gravity_torque);
}

state_representation::CartesianPose Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                              unsigned int frame_id) {
  return this->forward_kinematics(joint_positions, std::vector<unsigned int>{frame_id}).front();
}

std::vector<state_representation::CartesianPose> Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                                           const std::vector<unsigned int>& frame_ids) {
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw (exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints()));
  }
  std::vector<state_representation::CartesianPose> pose_vector;
  pinocchio::forwardKinematics(this->robot_model_, this->robot_data_, joint_positions.data());
  for (unsigned int id : frame_ids) {
    if (id >= static_cast<unsigned int>(this->robot_model_.nframes)) {
      throw (exceptions::FrameNotFoundException(std::to_string(id)));
    }
    pinocchio::updateFramePlacement(this->robot_model_, this->robot_data_, id);
    pinocchio::SE3 pose = this->robot_data_.oMf[id];
    Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond quaternion;
    pinocchio::quaternion::assignQuaternion(quaternion, pose.rotation());
    state_representation::CartesianPose frame_pose(this->robot_model_.frames[id].name,
                                                   translation,
                                                   quaternion,
                                                   this->get_base_frame());
    pose_vector.push_back(frame_pose);
  }
  return pose_vector;
}

state_representation::CartesianPose Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                              std::string frame_name) {
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_name = this->robot_model_.frames.back().name;
  }
  return this->forward_kinematics(joint_positions, std::vector<std::string>{frame_name}).front();
}

std::vector<state_representation::CartesianPose> Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                                           const std::vector<std::string>& frame_names) {
  std::vector<unsigned int> frame_ids(frame_names.size());
  for (unsigned int i = 0; i < frame_names.size(); ++i) {
    std::string name = frame_names[i];
    if (!this->robot_model_.existFrame(name)) {
      throw (exceptions::FrameNotFoundException(name));
    }
    frame_ids[i] = this->robot_model_.getFrameId(name);
  }
  return this->forward_kinematics(joint_positions, frame_ids);
}

Eigen::MatrixXd Model::cwln_weighted_matrix(const state_representation::JointPositions& joint_positions,
                                            const double margin) {
  Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(this->robot_model_.nq, this->robot_model_.nq);
  for (int n = 0; n < this->robot_model_.nq; ++n) {
    double d = 1;
    W_b(n, n) = 1;
    if (joint_positions.data()[n] < this->robot_model_.lowerPositionLimit[n] + margin) {
      if (joint_positions.data()[n] < this->robot_model_.lowerPositionLimit[n]) {
        W_b(n, n) = 0;
      } else {
        d = (this->robot_model_.lowerPositionLimit[n] + margin - joint_positions.data()[n]) / margin;
        W_b(n, n) = -2 * d * d * d + 3 * d * d;
      }
    } else if (this->robot_model_.upperPositionLimit[n] - margin < joint_positions.data()[n]) {
      if (this->robot_model_.upperPositionLimit[n] < joint_positions.data()[n]) {
        W_b(n, n) = 0;
      } else {
        d = (joint_positions.data()[n] - (this->robot_model_.upperPositionLimit[n] - margin)) / margin;
        W_b(n, n) = -2 * d * d * d + 3 * d * d;
      }
    }
  }
  return W_b;
}

Eigen::VectorXd Model::cwln_repulsive_potential_field(const state_representation::JointPositions& joint_positions,
                                                      double margin) {
  Eigen::VectorXd Psi(this->robot_model_.nq);
  Eigen::VectorXd q = joint_positions.data();
  for (int i = 0; i < this->robot_model_.nq; ++i) {
    Psi[i] = 0;
    if (q[i] < this->robot_model_.lowerPositionLimit[i] + margin) {
      Psi[i] = this->robot_model_.upperPositionLimit[i] - margin
          - std::max(q[i], this->robot_model_.lowerPositionLimit[i]);
    } else if (this->robot_model_.upperPositionLimit[i] - margin < q[i]) {
      Psi[i] = this->robot_model_.lowerPositionLimit[i] + margin
          - std::min(q[i], this->robot_model_.upperPositionLimit[i]);
    }
  }
  return Psi;
}

state_representation::JointPositions
Model::inverse_kinematics(const state_representation::CartesianPose& cartesian_pose,
                          const state_representation::JointPositions& joint_positions,
                          const std::string& frame_name,
                          const InverseKinematicsParameters& parameters) {
  unsigned int frame_id;
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_id = this->robot_model_.getFrameId(this->robot_model_.frames.back().name);
  } else {
    // throw error if specified frame does not exist
    if (!this->robot_model_.existFrame(frame_name)) {
      throw (exceptions::FrameNotFoundException(frame_name));
    }
    frame_id = this->robot_model_.getFrameId(frame_name);
  }
  std::string actual_frame_name = this->robot_model_.frames[frame_id].name;
  // 1 second for the Newton-Raphson method
  const std::chrono::nanoseconds dt(static_cast<int>(1e9));
  // initialization of the loop variables
  state_representation::JointPositions q(joint_positions);
  state_representation::JointVelocities dq(this->get_robot_name(), joint_positions.get_names());
  state_representation::Jacobian J(this->get_robot_name(),
                                   this->get_joint_frames(),
                                   actual_frame_name,
                                   this->get_base_frame());
  Eigen::MatrixXd J_b = Eigen::MatrixXd::Zero(6, this->get_number_of_joints());
  Eigen::MatrixXd JJt(6, 6);
  Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(this->get_number_of_joints(), this->get_number_of_joints());
  Eigen::MatrixXd W_c = Eigen::MatrixXd::Identity(this->get_number_of_joints(), this->get_number_of_joints());
  Eigen::VectorXd psi(this->get_number_of_joints());
  Eigen::VectorXd err(6);
  for (unsigned int i = 0; i < parameters.max_number_of_iterations; ++i) {
    err = ((cartesian_pose - this->forward_kinematics(q, frame_id)) / dt).data();
    // break in case of convergence
    if (err.cwiseAbs().maxCoeff() < parameters.tolerance) {
      return q;
    }
    J = this->compute_jacobian(q, actual_frame_name);
    W_b = this->cwln_weighted_matrix(q, parameters.margin);
    W_c = Eigen::MatrixXd::Identity(this->get_number_of_joints(), this->get_number_of_joints()) - W_b;
    psi = parameters.gamma * this->cwln_repulsive_potential_field(q, parameters.margin);
    J_b = J * W_b;
    JJt.noalias() = J_b * J_b.transpose();
    JJt.diagonal().array() += parameters.damp;
    dq.set_velocities(W_c * psi + parameters.alpha * W_b * (J_b.transpose() * JJt.ldlt().solve(err - J * W_c * psi)));
    q += dq * dt;
    q = this->clamp_in_range(q);
  }
  throw (exceptions::InverseKinematicsNotConvergingException(parameters.max_number_of_iterations,
                                                             err.array().abs().maxCoeff()));
}

state_representation::JointPositions
Model::inverse_kinematics(const state_representation::CartesianPose& cartesian_pose,
                          const std::string& frame_name,
                          const InverseKinematicsParameters& parameters) {
  Eigen::VectorXd q(pinocchio::neutral(this->robot_model_));
  state_representation::JointPositions positions(this->get_robot_name(), this->get_joint_frames(), q);
  return this->inverse_kinematics(cartesian_pose, positions, frame_name, parameters);
}

std::vector<state_representation::CartesianTwist>
Model::forward_velocity(const state_representation::JointState& joint_state,
                        const std::vector<std::string>& frame_names) {
  std::vector<state_representation::CartesianTwist> cartesian_twists(frame_names.size());
  for (std::size_t i = 0; i < frame_names.size(); ++i) {
    cartesian_twists.at(i) = this->compute_jacobian(joint_state, frame_names.at(i))
        * static_cast<state_representation::JointVelocities>(joint_state);
  }
  return cartesian_twists;
}

state_representation::CartesianTwist Model::forward_velocity(const state_representation::JointState& joint_state,
                                                             const std::string& frame_name) {
  return this->forward_velocity(joint_state, std::vector<std::string>{frame_name}).front();
}

state_representation::JointVelocities
Model::inverse_velocity(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                        const state_representation::JointPositions& joint_positions,
                        const InverseVelocityParameters& parameters) {
  this->init_qp_solver(parameters);
  const unsigned int nb_joints = this->get_number_of_joints();
  using namespace state_representation;
  // the velocity vector contains position of the intermediate frame and full pose of the end-effector
  Eigen::VectorXd delta_r(3 * cartesian_twists.size() + 3);
  Eigen::MatrixXd jacobian(3 * cartesian_twists.size() + 3, nb_joints);
  for (unsigned int i = 0; i < cartesian_twists.size() - 1; ++i) {
    CartesianTwist twist = cartesian_twists[i];
    // extract only the position for intermediate points
    delta_r.segment<3>(3 * i) = twist.get_linear_velocity();
    jacobian.block(3 * i, 0, 3 * i + 3, nb_joints) =
        this->compute_jacobian(joint_positions, twist.get_name()).data().block(0, 0, 3, nb_joints);
  }
  // extract the orientation for the end-effector
  CartesianTwist state = cartesian_twists.back();
  delta_r.segment<3>(3 * (cartesian_twists.size() - 1)) = state.get_linear_velocity();
  delta_r.tail(3) = state.get_angular_velocity();
  jacobian.bottomRows(6) = this->compute_jacobian(joint_positions, state.get_name()).data();
  // compute the Jacobian
  Eigen::MatrixXd hessian_matrix = jacobian.transpose() * jacobian;
  // set the hessian sparse matrix
  std::vector<Eigen::Triplet<double>> coefficients;
  for (unsigned int i = 0; i < nb_joints; ++i) {
    for (unsigned int j = 0; j < nb_joints; ++j) {
      coefficients.emplace_back(Eigen::Triplet<double>(i, j, hessian_matrix(i, j)));
    }
  }
  coefficients.emplace_back(Eigen::Triplet<double>(nb_joints, nb_joints, parameters.alpha));
  this->hessian_.setFromTriplets(coefficients.begin(), coefficients.end());
  //set the gradient
  this->gradient_.head(nb_joints) = -parameters.proportional_gain * delta_r.transpose() * jacobian;
  // update qp_constraints
  this->lower_bound_constraints_(4 * nb_joints) = parameters.epsilon;
  for (unsigned int i = 0; i < 3; ++i) {
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 1, nb_joints) = parameters.linear_velocity_limit;
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 4, nb_joints) = parameters.angular_velocity_limit;
  }
  // update the constraints
  this->solver_.updateHessianMatrix(this->hessian_);
  this->solver_.updateGradient(this->gradient_);
  this->solver_.updateBounds(this->lower_bound_constraints_, this->upper_bound_constraints_);
  this->solver_.updateLinearConstraintsMatrix(this->constraint_matrix_);
  // solve the QP problem
  this->solver_.solve();
  Eigen::VectorXd solution = this->solver_.getSolution();
  // extract the solution
  JointVelocities result(joint_positions.get_name(), joint_positions.get_names());
  Eigen::VectorXd delta_q = solution.head(nb_joints);
  //double T = solution(nb_joints);
  //result.set_velocities(delta_q / T);
  return result;
}

state_representation::JointVelocities Model::inverse_velocity(const state_representation::CartesianTwist& cartesian_twist,
                                                              const state_representation::JointPositions& joint_positions,
                                                              const InverseVelocityParameters& parameters) {
  return this->inverse_velocity(std::vector<state_representation::CartesianTwist>({cartesian_twist}),
                                joint_positions,
                                parameters);
}

void Model::print_qp_problem() {
  std::cout << "hessian:" << std::endl;
  std::cout << this->hessian_ << std::endl;

  for (unsigned int i = 0; i < this->constraint_matrix_.rows(); ++i) {
    std::cout << this->lower_bound_constraints_(i);
    std::cout << " < | ";
    for (unsigned int j = 0; j < this->constraint_matrix_.cols(); ++j) {
      std::cout << this->constraint_matrix_.coeffRef(i, j) << " | ";
    }
    std::cout << " < ";
    std::cout << this->upper_bound_constraints_(i) << std::endl;
  }
}

bool Model::in_range(const Eigen::VectorXd& vector,
                     const Eigen::VectorXd& lower_limits,
                     const Eigen::VectorXd& upper_limits) {
  return ((vector.array() >= lower_limits.array()).all() && (vector.array() <= upper_limits.array()).all());
}

bool Model::in_range(const state_representation::JointPositions& joint_positions) const {
  return this->in_range(joint_positions.get_positions(),
                        this->robot_model_.lowerPositionLimit,
                        this->robot_model_.upperPositionLimit);
}

bool Model::in_range(const state_representation::JointVelocities& joint_velocities) const {
  return this->in_range(joint_velocities.get_velocities(),
                        -this->robot_model_.velocityLimit,
                        this->robot_model_.velocityLimit);
}

bool Model::in_range(const state_representation::JointTorques& joint_torques) const {
  return this->in_range(joint_torques.get_torques(), -this->robot_model_.effortLimit, this->robot_model_.effortLimit);
}

bool Model::in_range(const state_representation::JointState& joint_state) const {
  return (this->in_range(static_cast<state_representation::JointPositions>(joint_state))
      && this->in_range(static_cast<state_representation::JointVelocities>(joint_state))
      && this->in_range(static_cast<state_representation::JointTorques>(joint_state)));
}

Eigen::VectorXd Model::clamp_in_range(const Eigen::VectorXd& vector,
                                      const Eigen::VectorXd& lower_limits,
                                      const Eigen::VectorXd& upper_limits) {
  return lower_limits.cwiseMax(upper_limits.cwiseMin(vector));
}

state_representation::JointState Model::clamp_in_range(const state_representation::JointState& joint_state) const {
  state_representation::JointState joint_state_clamped(joint_state);
  joint_state_clamped.set_positions(this->clamp_in_range(joint_state.get_positions(),
                                                         this->robot_model_.lowerPositionLimit,
                                                         this->robot_model_.upperPositionLimit));
  joint_state_clamped.set_velocities(this->clamp_in_range(joint_state.get_velocities(),
                                                          -this->robot_model_.velocityLimit,
                                                          this->robot_model_.velocityLimit));
  joint_state_clamped.set_torques(this->clamp_in_range(joint_state.get_torques(),
                                                       -this->robot_model_.effortLimit,
                                                       this->robot_model_.effortLimit));
  return joint_state_clamped;
}
}// namespace robot_model
