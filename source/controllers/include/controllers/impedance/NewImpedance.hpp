#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "controllers/IController.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/State.hpp"

namespace controllers::impedance {

/**
 * @class NewImpedance
 * @brief Definition of an impedance controller in either joint or task space
 * @tparam S the space of the controller (either CartesianState or JointState)
 */
template<class S>
class NewImpedance : public IController<S> {
public:

  /**
   * @brief Base constructor.
   * @details This initializes all gain matrices to the identity matrix of the corresponding dimensionality.
   * @param dimensions The number of dimensions associated with the controller
   */
  explicit NewImpedance(unsigned int dimensions = 6);

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial gain values
   * @param dimensions The number of dimensions associated with the controller
   */
  explicit NewImpedance(
      const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      unsigned int dimensions = 6
  );

  /**
   * @copydoc IController::compute_command(const S&,const S&)
   */
  S compute_command(const S& command_state, const S& feedback_state) override;

  /**
   * @brief Getter of the damping matrix
   * @return the damping matrix
   */
  [[nodiscard]] const Eigen::MatrixXd& get_damping() const;

  /**
   * @brief Getter of the stiffness matrix
   * @return the stiffness matrix
   */
  [[nodiscard]] const Eigen::MatrixXd& get_stiffness() const;

  /**
   * @brief Getter of the inertia matrix
   * @return the inertia matrix
   */
  [[nodiscard]] const Eigen::MatrixXd& get_inertia() const;

  /**
   * @brief Setter of the damping matrix
   * @param the new damping matrix
   */
  void set_damping(const Eigen::MatrixXd& damping);

  /**
   * @brief Setter of the stiffness matrix
   * @param the new stiffness matrix
   */
  void set_stiffness(const Eigen::MatrixXd& stiffness);

  /**
   * @brief Setter of the inertia matrix
   * @param the new inertia matrix
   */
  void set_inertia(const Eigen::MatrixXd& inertia);

protected:

  /**
   * @brief Validate and set parameters for damping, stiffness and inertia gain matrices.
   * @param parameter A parameter interface pointer
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Convert a parameterized gain to a gain matrix
   * @param parameter A parameter interface pointer of type double, double array or matrix
   * @return A square gain matrix
   */
  Eigen::MatrixXd
  gain_matrix_from_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>
      stiffness_; ///< stiffness matrix of the controller associated to position
  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>
      damping_; ///< damping matrix of the controller associated to velocity
  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>
      inertia_; ///< inertia matrix of the controller associated to acceleration

  const unsigned int dimensions_; ///< dimensionality of the control space and associated gain matricess
};

template<class S>
NewImpedance<S>::NewImpedance(unsigned int dimensions) :
    stiffness_(
        state_representation::make_shared_parameter<Eigen::MatrixXd>(
            "stiffness", Eigen::MatrixXd::Identity(dimensions, dimensions))), damping_(
    state_representation::make_shared_parameter<Eigen::MatrixXd>(
        "damping", Eigen::MatrixXd::Identity(dimensions, dimensions))), inertia_(
    state_representation::make_shared_parameter<Eigen::MatrixXd>(
        "inertia", Eigen::MatrixXd::Identity(dimensions, dimensions))), dimensions_(dimensions) {
  this->parameters_.insert(std::make_pair("stiffness", stiffness_));
  this->parameters_.insert(std::make_pair("damping", damping_));
  this->parameters_.insert(std::make_pair("inertia", inertia_));
}

template<class S>
NewImpedance<S>::NewImpedance(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters, unsigned int dimensions
) :
    NewImpedance(dimensions) {
  this->set_parameters(parameters);
}

template<class S>
void NewImpedance<S>::validate_and_set_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (parameter->get_name() == "stiffness") {
    this->stiffness_->set_value(this->gain_matrix_from_parameter(parameter));
  } else if (parameter->get_name() == "damping") {
    this->damping_->set_value(this->gain_matrix_from_parameter(parameter));
  } else if (parameter->get_name() == "inertia") {
    this->inertia_->set_value(this->gain_matrix_from_parameter(parameter));
  }
}

template<class S>
Eigen::MatrixXd NewImpedance<S>::gain_matrix_from_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  Eigen::MatrixXd matrix;
  if (parameter->get_type() == state_representation::StateType::PARAMETER_DOUBLE) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<double>>(parameter);
    matrix = gain->get_value() * Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
  } else if (parameter->get_type() == state_representation::StateType::PARAMETER_DOUBLE_ARRAY) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<std::vector<double>>>(parameter);
    if (gain->get_value().size() != this->dimensions_) {
      throw state_representation::exceptions::IncompatibleSizeException(
          "The provided diagonal coefficients do not match the dimensionality of the controller ("
              + std::to_string(this->dimensions_) + ")");
    }
    Eigen::VectorXd diagonal = Eigen::VectorXd::Map(gain->get_value().data(), this->dimensions_);
    matrix = diagonal.asDiagonal();
  } else if (parameter->get_type() == state_representation::StateType::PARAMETER_MATRIX) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<Eigen::MatrixXd>>(parameter);
    if (gain->get_value().rows() != this->dimensions_ || gain->get_value().cols() != this->dimensions_) {
      auto dim = std::to_string(this->dimensions_);
      throw state_representation::exceptions::IncompatibleSizeException(
          "The provided matrix does not have the expected size (" + dim + "x" + dim + ")");
    }
    matrix = gain->get_value();
  } else {
    throw state_representation::exceptions::InvalidParameterException(
        "Parameter " + parameter->get_name() + " has incorrect type");
  }
  return matrix;
}

template<class S>
const Eigen::MatrixXd& NewImpedance<S>::get_damping() const {
  return this->damping_->get_value();
}

template<class S>
const Eigen::MatrixXd& NewImpedance<S>::get_stiffness() const {
  return this->stiffness_->get_value();
}

template<class S>
const Eigen::MatrixXd& NewImpedance<S>::get_inertia() const {
  return this->inertia_->get_value();
}

template<class S>
inline void NewImpedance<S>::set_damping(const Eigen::MatrixXd& damping) {
  this->damping_->set_value(damping);
}

template<class S>
inline void NewImpedance<S>::set_stiffness(const Eigen::MatrixXd& stiffness) {
  this->stiffness_->set_value(stiffness);
}

template<class S>
inline void NewImpedance<S>::set_inertia(const Eigen::MatrixXd& inertia) {
  this->inertia_->set_value(inertia);
}

}// namespace controllers
