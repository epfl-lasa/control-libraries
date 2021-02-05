#include "robot_model/Model.hpp"
#include "robot_model/Exceptions/FrameNotFoundException.hpp"
#include "robot_model/Exceptions/InvalidJointStateSizeException.hpp"

namespace RobotModel {
Model::Model() : robot_name_(std::make_shared<StateRepresentation::Parameter<std::string>>("robot_name")),
                 urdf_path_(std::make_shared<StateRepresentation::Parameter<std::string>>("urdf_path")),
                 alpha_(std::make_shared<StateRepresentation::Parameter<double>>("alpha", 0.1)),
                 epsilon_(std::make_shared<StateRepresentation::Parameter<double>>("epsilon", 1e-2)),
                 linear_velocity_limit_(std::make_shared<StateRepresentation::Parameter<double>>("linear_velocity_limit",
                                                                                                 2.0)),
                 angular_velocity_limit_(std::make_shared<StateRepresentation::Parameter<double>>(
                     "angular_velocity_limit",
                     100.0)),
                 proportional_gain_(std::make_shared<StateRepresentation::Parameter<double>>("proportional_gain",
                                                                                             1.0)) {}

Model::Model(const std::string& robot_name, const std::string& urdf_path) : robot_name_(std::make_shared<
    StateRepresentation::Parameter<std::string>>("robot_name", robot_name)),
                                                                            urdf_path_(std::make_shared<
                                                                                StateRepresentation::Parameter<std::string>>(
                                                                                "urdf_path",
                                                                                urdf_path)),
                                                                            alpha_(std::make_shared<StateRepresentation::Parameter<
                                                                                double>>("alpha", 0.1)),
                                                                            epsilon_(std::make_shared<
                                                                                StateRepresentation::Parameter<double>>(
                                                                                "epsilon",
                                                                                1e-2)),
                                                                            linear_velocity_limit_(std::make_shared<
                                                                                StateRepresentation::Parameter<double>>(
                                                                                "linear_velocity_limit",
                                                                                2.0)),
                                                                            angular_velocity_limit_(std::make_shared<
                                                                                StateRepresentation::Parameter<double>>(
                                                                                "angular_velocity_limit",
                                                                                100.0)),
                                                                            proportional_gain_(std::make_shared<
                                                                                StateRepresentation::Parameter<double>>(
                                                                                "proportional_gain",
                                                                                1.0)) {
  this->init_model();
  this->init_qp_solver();
}

Model::~Model() {}

Model::Model(const Model& model) : robot_name_(model.robot_name_),
                                   urdf_path_(model.urdf_path_),
                                   alpha_(model.alpha_),
                                   epsilon_(model.epsilon_),
                                   linear_velocity_limit_(model.linear_velocity_limit_),
                                   angular_velocity_limit_(model.angular_velocity_limit_),
                                   proportional_gain_(model.proportional_gain_) {
  this->init_model();
  this->init_qp_solver();
}

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

bool Model::init_qp_solver() {
  size_t nb_joints = this->get_nb_joints();
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

  // joint dependant constraints
  for (size_t n = 0; n < nb_joints; ++n) {
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
  for (size_t i = 0; i < 3; ++i) {
    // linear velocity
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 1, nb_joints) = this->linear_velocity_limit_->get_value();
    this->upper_bound_constraints_(4 * nb_joints + i + 1) = std::numeric_limits<double>::infinity();
    // angular velocity
    this->constraint_matrix_.coeffRef(4 * nb_joints + i + 4, nb_joints) = this->angular_velocity_limit_->get_value();
    this->upper_bound_constraints_(4 * nb_joints + i + 4) = std::numeric_limits<double>::infinity();
  }

  // set the initial data of the QP solver_
  this->solver_.data()->setNumberOfVariables(nb_joints + 1);
  this->solver_.data()->setNumberOfConstraints(this->lower_bound_constraints_.size());
  if (!this->solver_.data()->setHessianMatrix(this->hessian_)) { return false; }
  if (!this->solver_.data()->setGradient(this->gradient_)) { return false; }
  if (!this->solver_.data()->setLinearConstraintsMatrix(this->constraint_matrix_)) { return false; }
  if (!this->solver_.data()->setLowerBound(this->lower_bound_constraints_)) { return false; }
  if (!this->solver_.data()->setUpperBound(this->upper_bound_constraints_)) { return false; }
  // instantiate the solver_
  return this->solver_.initSolver();
}

StateRepresentation::Jacobian Model::compute_jacobian(const StateRepresentation::JointState& joint_state,
                                                      int frame_id) {
  if ((int) joint_state.get_size() != this->robot_model_.nq) {
    throw (Exceptions::InvalidJointStateSizeException(joint_state.get_size(), this->robot_model_.nq));
  }
  // compute the jacobian from the joint state
  pinocchio::Data::Matrix6x J(6, this->robot_model_.nq);
  J.setZero();
  pinocchio::computeFrameJacobian(this->robot_model_,
                                  this->robot_data_,
                                  joint_state.get_positions(),
                                  frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED,
                                  J);
  return StateRepresentation::Jacobian(this->get_robot_name(), J);
}

StateRepresentation::Jacobian Model::compute_jacobian(const StateRepresentation::JointState& joint_state,
                                                      const std::string& frame_name) {
  int frame_id;
  if (frame_name.empty()) {
    // get last frame if none specified
    frame_id = this->robot_model_.getFrameId(this->robot_model_.frames.back().name);
  } else {
    // throw error if specified frame does not exist
    if (!this->robot_model_.existFrame(frame_name)) { throw (Exceptions::FrameNotFoundException(frame_name)); }
    frame_id = this->robot_model_.getFrameId(frame_name);
  }
  std::cout << frame_id << std::endl;
  return this->compute_jacobian(joint_state, frame_id);
}

std::vector<StateRepresentation::CartesianPose> Model::forward_geometry(const StateRepresentation::JointState& joint_state,
                                                                        const std::vector<unsigned int>& frame_ids) {
  if ((int) joint_state.get_size() != this->robot_model_.nq) {
    throw (Exceptions::InvalidJointStateSizeException(joint_state.get_size(), this->robot_model_.nq));
  }
  std::vector<StateRepresentation::CartesianPose> pose_vector;
  pinocchio::forwardKinematics(this->robot_model_, this->robot_data_, joint_state.get_positions());
  for (unsigned int id : frame_ids) {
    pinocchio::updateFramePlacement(this->robot_model_, this->robot_data_, id);
    pinocchio::SE3 pose = this->robot_data_.oMf[id];
    Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond quaternion;
    pinocchio::quaternion::assignQuaternion(quaternion, pose.rotation());
    StateRepresentation::CartesianPose frame_pose(this->robot_model_.frames[id].name, translation, quaternion);
    pose_vector.push_back(frame_pose);
  }
  return pose_vector;
}

std::vector<StateRepresentation::CartesianPose> Model::forward_geometry(const StateRepresentation::JointState& joint_state,
                                                                        const std::vector<std::string>& frame_names) {
  std::vector<unsigned int> frame_ids(frame_names.size());
  for (std::size_t i = 0; i < frame_names.size(); ++i) {
    std::string name = frame_names[i];
    if (!this->robot_model_.existFrame(name)) { throw (Exceptions::FrameNotFoundException(name)); }
    frame_ids[i] = this->robot_model_.getFrameId(name);
  }
  return this->forward_geometry(joint_state, frame_ids);
}

StateRepresentation::CartesianPose Model::forward_geometry(const StateRepresentation::JointState& joint_state,
                                                           const std::string& frame_name) {
  return this->forward_geometry(joint_state, std::vector<std::string>{frame_name}).front();
}

const StateRepresentation::JointPositions Model::inverse_geometry(const StateRepresentation::CartesianState&) const {
  // TODO
  return StateRepresentation::JointPositions();
}

const StateRepresentation::CartesianTwist Model::forward_kinematic(const StateRepresentation::JointState& joint_state) {
  return this->compute_jacobian(joint_state) * static_cast<StateRepresentation::JointVelocities>(joint_state);
}

StateRepresentation::JointVelocities Model::inverse_kinematic(const StateRepresentation::JointState& joint_state,
                                                              const std::vector<StateRepresentation::CartesianState>& cartesian_states) {
  const size_t nb_joints = this->get_nb_joints();
  using namespace StateRepresentation;
  // the velocity vector contains position of the intermediate frame and full pose of the end-effector
  Eigen::VectorXd delta_r(3 * cartesian_states.size() + 3);
  Eigen::MatrixXd jacobian(3 * cartesian_states.size() + 3, nb_joints);

  for (size_t i = 0; i < cartesian_states.size() - 1; ++i) {
    CartesianState state = cartesian_states[i];
    // extract only the position for intermediate points
    delta_r.segment<3>(3 * i) = state.get_linear_velocity();
    jacobian.block(3 * i, 0, 3 * i + 3, nb_joints) =
        this->compute_jacobian(joint_state, state.get_name()).get_data().block(0, 0, 3, nb_joints);
  }
  // extract the orientation for the end-effector
  CartesianState state = cartesian_states.back();
  delta_r.segment<3>(3 * (cartesian_states.size() - 1)) = state.get_linear_velocity();
  delta_r.tail(3) = state.get_angular_velocity();
  jacobian.bottomRows(6) = this->compute_jacobian(joint_state, state.get_name()).get_data();
  // compute the Jacobian
  Eigen::MatrixXd hessian_matrix = jacobian.transpose() * jacobian;

  // set the hessian sparse matrix
  std::vector<Eigen::Triplet<double>> coefficients;
  for (size_t i = 0; i < nb_joints; ++i) {
    for (size_t j = 0; j < nb_joints; ++j) {
      coefficients.push_back(Eigen::Triplet<double>(static_cast<int>(i), static_cast<int>(j), hessian_matrix(i, j)));
    }
  }
  coefficients.push_back(Eigen::Triplet<double>(static_cast<int>(nb_joints),
                                                static_cast<int>(nb_joints),
                                                this->alpha_->get_value()));
  this->hessian_.setFromTriplets(coefficients.begin(), coefficients.end());

  //set the gradient
  this->gradient_.head(nb_joints) = -this->proportional_gain_->get_value() * delta_r.transpose() * jacobian;

  // update qp_constraints
  this->lower_bound_constraints_(4 * nb_joints) = this->epsilon_->get_value();
  for (size_t i = 0; i < 3; ++i) {
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

StateRepresentation::JointVelocities Model::inverse_kinematic(const StateRepresentation::JointState& joint_state,
                                                              const StateRepresentation::CartesianState& cartesian_state) {
  return this->inverse_kinematic(joint_state, std::vector<StateRepresentation::CartesianState>({cartesian_state}));
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
}// namespace RobotModel