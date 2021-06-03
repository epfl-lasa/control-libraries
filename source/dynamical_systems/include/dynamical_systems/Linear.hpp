/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/robot/JointPositions.hpp"
#include "state_representation/robot/JointState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"

using namespace state_representation;

namespace dynamical_systems {
/**
 * @class Linear
 * @brief Represent a Linear dynamical system to move toward an attractor
 * @tparam S the type of space of the dynamical system (e.g. Cartesian or Joint)
 */
template<class S>
class Linear : public DynamicalSystem<S> {
private:
  std::shared_ptr<Parameter<S>> attractor_;         ///< attractor of the dynamical system in the space
  std::shared_ptr<Parameter<Eigen::MatrixXd>> gain_;///< gain associate to the system

protected:
  /**
   * @brief Compute the dynamics of the input state.
   * Internal function, to be redefined based on the
   * type of dynamical system, called by the evaluate
   * function
   * @param state the input state
   * @return the output state
   */
  S compute_dynamics(const S& state) const override;

public:
  /**
   * @brief Empty constructor
   */
  Linear();

  /**
   * @brief Constructor with specified attractor and iso gain
   * @param attractor the attractor of the linear system
   * @param iso_gain the iso gain of the system
   */
  Linear(const S& attractor, double iso_gain = 1.0);

  /**
   * @brief Constructor with specified attractor and different gains specified as diagonal coefficients
   * @param attractor the attractor of the linear system
   * @param diagonal_coefficients the gains values specified as diagonal coefficients
   */
  Linear(const S& attractor, const std::vector<double>& diagonal_coefficients);

  /**
   * @brief Constructor with specified attractor and different gains specified as full gain matrix
   * @param attractor the attractor of the linear system
   * @param gain_matrix the gains values specified as a full matrix
   */
  Linear(const S& attractor, const Eigen::MatrixXd& gain_matrix);

  /**
   * @brief Getter of the attractor
   * @return the attractor as a const reference
   */
  const S& get_attractor() const;

  /**
   * @brief Setter of the attractor as a new value
   * @param attractor the new attractor
   */
  void set_attractor(const S& attractor);

  /**
   * @brief Setter of the base frame as a new value
   * @param base_frame the new base frame
   */
  void set_base_frame(const S& base_frame) override;

  /**
   * @brief Getter of the gain attribute
   * @return The gain value
   */
  const Eigen::MatrixXd& get_gain() const;

  /**
   * @brief Setter of the gain attribute
   * @param iso_gain the gain value as an iso coefficient
   */
  void set_gain(double iso_gain);

  /**
   * @brief Setter of the gain attribute
   * @param diagonal_coefficients the gain values as diagonal coefficients
   */
  void set_gain(const std::vector<double>& diagonal_coefficients);

  /**
   * @brief Setter of the gain attribute
   * @param gain_matrix the gain values as a full gain matrix
   */
  void set_gain(const Eigen::MatrixXd& gain_matrix);

  /**
   * @brief Return a list of all the parameters of the dynamical system
   * @return the list of parameters
   */
  std::list<std::shared_ptr<ParameterInterface>> get_parameters() const override;
};

template<>
void Linear<JointState>::set_gain(double iso_gain) {
  int nb_joints = this->get_attractor().get_size();
  this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(nb_joints, nb_joints));
}

template<class S>
inline const S& Linear<S>::get_attractor() const {
  return this->attractor_->get_value();
}

template<class S>
inline void Linear<S>::set_attractor(const S& attractor) {
  this->attractor_->set_value(attractor);
}

template<>
inline void Linear<CartesianState>::set_attractor(const CartesianState& attractor) {
  if (attractor.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(attractor.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    DynamicalSystem<CartesianState>::set_base_frame(CartesianState::Identity(attractor.get_reference_frame(),
                                                                             attractor.get_reference_frame()));
  }
  // validate that the reference frame of the attractor is always compatible with the DS reference frame
  if (attractor.get_reference_frame() != this->get_base_frame().get_name()) {
    if (attractor.get_reference_frame() != this->get_base_frame().get_reference_frame()) {
      throw state_representation::exceptions::IncompatibleReferenceFramesException(
          "The reference frame of the attractor " + attractor.get_name() + " in frame "
              + attractor.get_reference_frame() + " is incompatible with the base frame of the dynamical system "
              + this->get_base_frame().get_name() + " in frame " + this->get_base_frame().get_reference_frame() + ".");
    }
    this->attractor_->set_value(this->get_base_frame().inverse() * attractor);
  } else {
    this->attractor_->set_value(attractor);
  }
}

template<>
inline void Linear<JointState>::set_attractor(const JointState& attractor) {
  if (attractor.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(attractor.get_name() + " state is empty");
  }
  if (this->get_base_frame().is_empty()) {
    DynamicalSystem<JointState>::set_base_frame(JointState::Zero(attractor.get_name(), attractor.get_names()));
  }
  // validate that the attractor is compatible with the DS reference name
  if (!this->is_compatible(attractor)) {
    throw state_representation::exceptions::IncompatibleReferenceFramesException(
        "The attractor " + attractor.get_name() + " is incompatible with the base frame of the dynamical system "
            + this->get_base_frame().get_name() + ".");
  }
  this->attractor_->set_value(attractor);
  if (this->get_gain().size() == 0) {
    this->set_gain(1);
  }
}

template<class S>
inline void Linear<S>::set_base_frame(const S& base_frame) {
  DynamicalSystem<S>::set_base_frame(base_frame);
}

template<>
inline void Linear<CartesianState>::set_base_frame(const CartesianState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  DynamicalSystem<CartesianState>::set_base_frame(base_frame);
  if (!this->get_attractor().is_empty()) {
    // update reference frame of attractor
    auto attractor = this->get_attractor();
    attractor.set_reference_frame(base_frame.get_name());
    this->set_attractor(attractor);
  }
}

template<>
inline void Linear<JointState>::set_base_frame(const JointState& base_frame) {
  if (base_frame.is_empty()) {
    throw state_representation::exceptions::EmptyStateException(base_frame.get_name() + " state is empty");
  }
  DynamicalSystem<JointState>::set_base_frame(base_frame);
  if (!this->get_attractor().is_empty()) {
    // update name of attractor
    auto attractor = this->get_attractor();
    attractor.set_name(base_frame.get_name());
    attractor.set_names(base_frame.get_names());
    this->set_attractor(attractor);
  }
}

template<class S>
inline const Eigen::MatrixXd& Linear<S>::get_gain() const {
  return this->gain_->get_value();
}

template<class S>
std::list<std::shared_ptr<ParameterInterface>> Linear<S>::get_parameters() const {
  std::list<std::shared_ptr<ParameterInterface>> param_list;
  param_list.push_back(this->attractor_);
  param_list.push_back(this->gain_);
  return param_list;
}
}// namespace dynamical_systems
