/**
 * @author Baptiste Busch
 * @date 2019/07/18
 *
 */

#pragma once

#include "dynamical_systems/DynamicalSystem.hpp"
#include "dynamical_systems/Exceptions/NotImplementedException.hpp"
#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"

namespace DynamicalSystems {
/**
 * @class Blended
 * @brief Represent a blend of several dynamical systems where the output is the weighted sum of the output of each system
 * @tparam S the type of space of the dynamical system (e.g. Cartesian or Joint)
 */
template <class S>
class Blending : public DynamicalSystem<S> {
private:
  std::vector<std::shared_ptr<DynamicalSystem<S>>> systems_;                    ///< pointer to the individual systmes
  std::shared_ptr<StateRepresentation::Parameter<std::vector<double>>> weights_;///< weights of each system

  /**
   * @brief Normalize the weight vector
   */
  void normalize_weights();

protected:
  /**
   * @brief Compute the dynamics of the input state.
   * Internal function, to be redefined based on the
   * type of dynamical system, called by the evaluate
   * function
   * @param state the input state
   * @return the output state
   */
  S compute_dynamics(const S& state) const;

public:
  /**
   * @brief Empty coinstructor
   */
  explicit Blending();

  /**
   * @brief Getter of the dynamical system vector
   * @return the dynamical system vector
   */
  const std::vector<std::shared_ptr<DynamicalSystem<S>>>& get_dynamical_systems() const;

  /**
   * @brief Getter of the dynamical system at desired index
   * @param idx the index of the dynamical system
   * @return the desired dynamical system
   */
  const std::shared_ptr<DynamicalSystem<S>>& get_dynamical_systems(unsigned int idx) const;

  /**
   * @brief Add a dynamical system to the blending
   * @param ds the dynamical system to add
   * @weight the weight associated to the ds
   * @normalize_weights if true normalize the weight vector after adding the system
   */
  void add_dynamical_system(const std::shared_ptr<DynamicalSystem<S>> ds, double weight, bool normalize_weights = true);

  /**
   * @brief Add several dynamical system to the blending
   * @param ds the dynamical systems to add
   * @param weights the weight associated to the ds
   * @normalize_weights if true normalize the weight vector after adding the system
   */
  void add_dynamical_systems(const std::vector<std::shared_ptr<DynamicalSystem<S>>>& ds, const std::vector<double>& weight, bool normalize_weights = true);

  /**
   * @brief Set the weight of the dynamical system at desired index
   * @param idx index of the dynamical system
   * @param weight the new weight value
   * @normalize_weights if true normalize the weight vector after adding the system
   */
  void set_weight(unsigned int idx, double weight, bool normalize_weights = true);

  /**
   * @brief Set the weights of the dynamical system at desired indexes
   * @param idx indexes of the dynamical system
   * @param weights the new weight value
   * @normalize_weights if true normalize the weight vector after adding the system
   */
  void set_weights(const std::vector<unsigned int>& idx, const std::vector<double>& weights, bool normalize_weights = true);
};

template <class S>
Blending<S>::Blending() : DynamicalSystem<S>() {}

template <class S>
void Blending<S>::normalize_weights() {
  double mod = 0.0;
  for (const auto& w : this->weights_->get_value()) mod += w * w;
  double mag = std::sqrt(mod);
  for (auto& w : this->weights_->get_value()) w /= mag;
}

template <class S>
inline const std::vector<std::shared_ptr<DynamicalSystem<S>>>& Blending<S>::get_dynamical_systems() const {
  return this->systems_;
}

template <class S>
const std::shared_ptr<DynamicalSystem<S>>& Blending<S>::get_dynamical_systems(unsigned int idx) const {
  return this->systems_[idx];
}

template <class S>
void Blending<S>::add_dynamical_system(const std::shared_ptr<DynamicalSystem<S>> ds, double weight, bool normalize_weights = true) {
  this->systems_.push_back(ds);
  this->weights_->get_value().push_back(weight);
  if (normalize_weights) this->normalize_weights();
}

template <class S>
void Blending<S>::add_dynamical_systems(const std::vector<std::shared_ptr<DynamicalSystem<S>>>& ds, const std::vector<double>& weights, bool normalize_weights = true) {
  if (ds.size() != weights.size()) throw IncompatibleSizeException("The dynamical system and the weight vector have different sizes");
  for (unsigned int i = 0; i < ds.size(); ++i) {
    this->systems_.push_back(ds);
    this->weights_->get_value().push_back(weight);
  }
  if (normalize_weights) this->normalize_weights();
}

template <class S>
void Blending<S>::set_weight(unsigned int idx, double weight, bool normalize_weights = true) {
  this->weights_->get_value()[idx] = weight;
  if (normalize_weights) this->normalize_weights();
}

template <class S>
void Blending<S>::set_weights(const std::vector<unsigned int>& idx, const std::vector<double>& weights, bool normalize_weights = true) {
  if (idx.size() != weights.size()) throw IncompatibleSizeException("The index vector and the weight vector have different sizes");
  for (unsigned int i = 0; i < idx.size(); ++i) {
    this->weights_->get_value()[idx[i]] = weights[i];
  }
  if (normalize_weights) this->normalize_weights();
}

template <class S>
const S Blending<S>::compute_dynamics(const S& state) const {
  S output;
  for (unsigned int i = 0; i < ds.size(); ++i) {
    S += (this->weights_->get_value()[i] * this->systems_[i]->evaluate(state));
  }
  return output;
}
}// namespace DynamicalSystems
