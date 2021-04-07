#include "robot_model/Model.hpp"
#include <fstream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "robot_model/exceptions/FrameNotFoundException.hpp"
#include "robot_model/exceptions/InvalidJointStateSizeException.hpp"
#include "robot_model/exceptions/IKDoesNotConverge.hpp"
#include <math.h>

namespace robot_model {
Model::Model(const std::string& robot_name, const std::string& urdf_path) :
    robot_name_(std::make_shared<state_representation::Parameter<std::string>>("robot_name", robot_name)),
    urdf_path_(std::make_shared<state_representation::Parameter<std::string>>("urdf_path", urdf_path)),
    alpha_(std::make_shared<state_representation::Parameter<double>>("alpha", 0.1)),
    epsilon_(std::make_shared<state_representation::Parameter<double>>("epsilon", 1e-2)),
    linear_velocity_limit_(std::make_shared<state_representation::Parameter<double>>("linear_velocity_limit", 2.0)),
    angular_velocity_limit_(std::make_shared<state_representation::Parameter<double>>("angular_velocity_limit", 100.0)),
    proportional_gain_(std::make_shared<state_representation::Parameter<double>>("proportional_gain", 1.0)) {
  this->init_model();
  this->init_qp_solver();
}

Model::Model(const Model& model) :
    robot_name_(model.robot_name_),
    urdf_path_(model.urdf_path_),
    alpha_(model.alpha_),
    epsilon_(model.epsilon_),
    linear_velocity_limit_(model.linear_velocity_limit_),
    angular_velocity_limit_(model.angular_velocity_limit_),
    proportional_gain_(model.proportional_gain_) {
  this->init_model();
  this->init_qp_solver();
}

Model::~Model() {}

Model& Model::operator=(const Model& model) {
  this->robot_name_ = model.robot_name_;
  this->urdf_path_ = model.urdf_path_;
  this->alpha_ = model.alpha_;
  this->epsilon_ = model.epsilon_;
  // initialize the model and the solver
  this->init_model();
  this->init_qp_solver();
  return (*this);
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

bool Model::init_qp_solver() {
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
  this->hessian_.coeffRef(nb_joints, nb_joints) = this->alpha_->get_value();
  this->constraint_matrix_.coeffRef(4 * nb_joints, nb_joints) = 1.0;
  this->lower_bound_constraints_(4 * nb_joints) = this->epsilon_->get_value();
  this->upper_bound_constraints_(4 * nb_joints) = std::numeric_limits<double>::infinity();
  // cartesian velocity constraints
  for (unsigned int i = 0; i < 3; ++i) {
    // linear velocity
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 1, nb_joints) = this->linear_velocity_limit_->get_value();
    this->upper_bound_constraints_(4 * nb_joints + i + 1) = std::numeric_limits<double>::infinity();
    // angular velocity
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 4, nb_joints) = this->angular_velocity_limit_->get_value();
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

state_representation::Jacobian Model::compute_jacobian(const state_representation::JointState& joint_state,
                                                       unsigned int frame_id) {
  if (joint_state.get_size() != this->get_number_of_joints()) {
    throw (exceptions::InvalidJointStateSizeException(joint_state.get_size(), this->get_number_of_joints()));
  }
  // compute the jacobian from the joint state
  pinocchio::Data::Matrix6x J(6, this->get_number_of_joints());
  J.setZero();
  pinocchio::computeFrameJacobian(this->robot_model_,
                                  this->robot_data_,
                                  joint_state.get_positions(),
                                  frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED,
                                  J);
  // the model does not have any reference frame
  return state_representation::Jacobian(this->get_robot_name(),
                                        this->get_joint_frames(),
                                        this->robot_model_.frames[frame_id].name,
                                        J,
                                        this->frame_names_.front());
}

state_representation::Jacobian Model::compute_jacobian(const state_representation::JointState& joint_state,
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
  return this->compute_jacobian(joint_state, frame_id);
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

state_representation::CartesianPose Model::forward_geometry(const state_representation::JointState& joint_state,
                                                            unsigned int frame_id) {
  return this->forward_geometry(joint_state, std::vector<unsigned int>{frame_id}).front();
}

std::vector<state_representation::CartesianPose> Model::forward_geometry(const state_representation::JointState& joint_state,
                                                                         const std::vector<unsigned int>& frame_ids) {
  if (joint_state.get_size() != this->get_number_of_joints()) {
    throw (exceptions::InvalidJointStateSizeException(joint_state.get_size(), this->get_number_of_joints()));
  }
  std::vector<state_representation::CartesianPose> pose_vector;
  pinocchio::forwardKinematics(this->robot_model_, this->robot_data_, joint_state.get_positions());
  for (unsigned int id : frame_ids) {
    if (id >= static_cast<unsigned int>(this->robot_model_.nframes)) {
      throw (exceptions::FrameNotFoundException(std::to_string(id)));
    }
    pinocchio::updateFramePlacement(this->robot_model_, this->robot_data_, id);
    pinocchio::SE3 pose = this->robot_data_.oMf[id];
    Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond quaternion;
    pinocchio::quaternion::assignQuaternion(quaternion, pose.rotation());
    state_representation::CartesianPose frame_pose(this->robot_model_.frames[id].name, translation, quaternion);
    pose_vector.push_back(frame_pose);
  }
  return pose_vector;
}

state_representation::CartesianPose Model::forward_geometry(const state_representation::JointState& joint_state,
                                                            std::string frame_name) {
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_name = this->robot_model_.frames.back().name;
  }
  return this->forward_geometry(joint_state, std::vector<std::string>{frame_name}).front();
}

std::vector<state_representation::CartesianPose> Model::forward_geometry(const state_representation::JointState& joint_state,
                                                                         const std::vector<std::string>& frame_names) {
  std::vector<unsigned int> frame_ids(frame_names.size());
  for (unsigned int i = 0; i < frame_names.size(); ++i) {
    std::string name = frame_names[i];
    if (!this->robot_model_.existFrame(name)) {
      throw (exceptions::FrameNotFoundException(name));
    }
    frame_ids[i] = this->robot_model_.getFrameId(name);
  }
  return this->forward_geometry(joint_state, frame_ids);
}

Eigen::MatrixXd Model::cwln_weighted_matrix(const state_representation::JointPositions& joint_positions, const double margin) {
  Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(this->robot_model_.nq, this->robot_model_.nq);
  for (int n = 0; n < this->robot_model_.nq; n++) {
    double d = 1;
    W_b(n, n) = 1;
    if (joint_positions.get_positions()[n] < this->robot_model_.lowerPositionLimit[n] + margin) {
      if (joint_positions.get_positions()[n] < this->robot_model_.lowerPositionLimit[n]) {
        W_b(n, n) = 0;
      } else {
        d = (this->robot_model_.lowerPositionLimit[n] + margin - joint_positions.get_positions()[n]) / margin;
        W_b(n, n) = -2 * d * d * d + 3 * d * d;
      }
    } else if (this->robot_model_.upperPositionLimit[n] - margin < joint_positions.get_positions()[n]) {
      if (this->robot_model_.upperPositionLimit[n] < joint_positions.get_positions()[n]) {
        W_b(n, n) = 0;
      } else {
        d = (joint_positions.get_positions()[n] - (this->robot_model_.upperPositionLimit[n] - margin)) / margin;
        W_b(n, n) = -2 * d * d * d + 3 * d * d;
      }
    }
  }
  return W_b;
}

Eigen::VectorXd Model::cwln_repulsive_potential_field(const state_representation::JointPositions& joint_positions, double margin) {
  Eigen::VectorXd Psi(this->robot_model_.nq);
  Eigen::VectorXd q = joint_positions.get_positions();
  for (int i = 0; i < this->robot_model_.nq; i++) {
    Psi[i] = 0;
    if(q[i] < this->robot_model_.lowerPositionLimit[i] + margin) {
      Psi[i] = this->robot_model_.upperPositionLimit[i] - margin - std::max(q[i], this->robot_model_.lowerPositionLimit[i]);
    } else if(this->robot_model_.upperPositionLimit[i] - margin < q[i]) {
      Psi[i] = this->robot_model_.lowerPositionLimit[i] + margin - std::min(q[i], this->robot_model_.upperPositionLimit[i]);
    }     
  }
  
  return Psi;
}

state_representation::JointPositions Model::inverse_geometry(const state_representation::CartesianState& desired_cartesian_state,
                                                              const state_representation::JointState& current_joint_state,
                                                              std::string frame_name,
                                                              const Model::InverseGeometryParameters& params) {

  if (frame_name.empty()) {
    // get last frame if none specified
    frame_name = this->robot_model_.frames.back().name;
  } else {
    // throw error if specified frame does not exist
    if (!this->robot_model_.existFrame(frame_name)) {
      throw (exceptions::FrameNotFoundException(frame_name));
    }
  }

  const int frame_id = this->robot_model_.getFrameId(frame_name);
  
  // 1 second for the Newton-Raphson method
  const std::chrono::nanoseconds dt((int)(1e9));

  state_representation::JointPositions q(current_joint_state);
  state_representation::JointVelocities dq(this->get_robot_name(), this->get_number_of_joints());
  state_representation::Jacobian J(this->get_robot_name(), this->robot_model_.nq, frame_name);
  Eigen::MatrixXd J_b = J.data();
  Eigen::MatrixXd JJt(6,6);
  Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(this->robot_model_.nq, this->robot_model_.nq);
  Eigen::MatrixXd W_c = Eigen::MatrixXd::Identity(this->robot_model_.nq, this->robot_model_.nq);
  Eigen::VectorXd psi(this->robot_model_.nq);
  Eigen::VectorXd err(6);
  state_representation::CartesianPose X(this->get_robot_name());
  const state_representation::CartesianPose X_d(desired_cartesian_state);
  
  int i = 0;
  while (i < params.max_number_of_iterations) {
    X = this->forward_geometry(q, frame_id);
    
    err = ((X_d - X)/dt).data();

    if(err.array().abs().maxCoeff() < params.tolerance) {
      return q;
    }

    J = this->compute_jacobian(q);
    
    W_b = this->cwln_weighted_matrix(q, params.margin);
    W_c = Eigen::MatrixXd::Identity(this->robot_model_.nq, this->robot_model_.nq) - W_b;
    psi = params.gamma * this->cwln_repulsive_potential_field(q, params.margin);
    J_b = J.data() * W_b;
    
    JJt.noalias() = J_b * J_b.transpose();
    JJt.diagonal().array() += params.damp;
    dq.set_velocities(W_c * psi + params.alpha * W_b * ( J_b.transpose() * JJt.ldlt().solve(err - J.data() * W_c * psi)));
    
    q = q + dq * dt;
    q = this->clamp_in_range(q);

    i++;
  }
  
  throw (exceptions::IKDoesNotConverge(params.max_number_of_iterations, err.array().abs().maxCoeff()));
}


state_representation::JointPositions Model::inverse_geometry(const state_representation::CartesianState& desired_cartesian_state,
                                                            std::string frame_name, const Model::InverseGeometryParameters& params) {
  Eigen::VectorXd q( pinocchio::neutral(this->robot_model_));
  state_representation::JointPositions pos(this->get_robot_name(),q);
  
  return this->inverse_geometry(desired_cartesian_state, pos, frame_name, params);
}

state_representation::CartesianTwist Model::forward_kinematic(const state_representation::JointState& joint_state) {
  return this->compute_jacobian(joint_state) * static_cast<state_representation::JointVelocities>(joint_state);
}

state_representation::JointVelocities Model::inverse_kinematic(const state_representation::JointState& joint_state,
                                                               const std::vector<state_representation::CartesianTwist>& cartesian_twist) {
  const unsigned int nb_joints = this->get_number_of_joints();
  using namespace state_representation;
  // the velocity vector contains position of the intermediate frame and full pose of the end-effector
  Eigen::VectorXd delta_r(3 * cartesian_twist.size() + 3);
  Eigen::MatrixXd jacobian(3 * cartesian_twist.size() + 3, nb_joints);

  for (unsigned int i = 0; i < cartesian_twist.size() - 1; ++i) {
    CartesianTwist twist = cartesian_twist[i];
    // extract only the position for intermediate points
    delta_r.segment<3>(3 * i) = twist.get_linear_velocity();
    jacobian.block(3 * i, 0, 3 * i + 3, nb_joints) =
        this->compute_jacobian(joint_state, twist.get_name()).data().block(0, 0, 3, nb_joints);
  }
  // extract the orientation for the end-effector
  CartesianTwist state = cartesian_twist.back();
  delta_r.segment<3>(3 * (cartesian_twist.size() - 1)) = state.get_linear_velocity();
  delta_r.tail(3) = state.get_angular_velocity();
  jacobian.bottomRows(6) = this->compute_jacobian(joint_state, state.get_name()).data();
  // compute the Jacobian
  Eigen::MatrixXd hessian_matrix = jacobian.transpose() * jacobian;

  // set the hessian sparse matrix
  std::vector<Eigen::Triplet<double>> coefficients;
  for (unsigned int i = 0; i < nb_joints; ++i) {
    for (unsigned int j = 0; j < nb_joints; ++j) {
      coefficients.emplace_back(Eigen::Triplet<double>(i, j, hessian_matrix(i, j)));
    }
  }
  coefficients.emplace_back(Eigen::Triplet<double>(nb_joints, nb_joints, this->alpha_->get_value()));
  this->hessian_.setFromTriplets(coefficients.begin(), coefficients.end());

  //set the gradient
  this->gradient_.head(nb_joints) = -this->proportional_gain_->get_value() * delta_r.transpose() * jacobian;

  // update qp_constraints
  this->lower_bound_constraints_(4 * nb_joints) = this->epsilon_->get_value();
  for (unsigned int i = 0; i < 3; ++i) {
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 1, nb_joints) = this->linear_velocity_limit_->get_value();
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 4, nb_joints) = this->angular_velocity_limit_->get_value();
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
  JointVelocities result(joint_state);
  Eigen::VectorXd delta_q = solution.head(nb_joints);
  double T = solution(nb_joints);
  result.set_velocities(delta_q / T);
  return result;
}

state_representation::JointVelocities Model::inverse_kinematic(const state_representation::JointState& joint_state,
                                                               const state_representation::CartesianTwist& cartesian_state) {
  return this->inverse_kinematic(joint_state, std::vector<state_representation::CartesianTwist>({cartesian_state}));
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
