#pragma once

#include "state_representation/State.hpp"

namespace state_representation {
class SpatialState : public State {
private:
  std::string reference_frame_; ///< name of the reference frame

public:
  /**
   * @brief Empty constructor only specifying the type.
   */
  explicit SpatialState(const StateType& type);

  /**
   * @brief Constructor with name and reference frame specification.
   * @param type The type of SpatialState (Cartesian or DualQuaternion)
   * @param name The name of the State
   * @param reference_frame The reference frame in which the state is expressed, by default world
   * @param empty Specify if the state is initialized as empty, default true
   */
  explicit SpatialState(
      const StateType& type, const std::string& name, const std::string& reference_frame = "world",
      const bool& empty = true
  );

  /**
   * @brief Copy constructor from another SpatialState.
   */
  SpatialState(const SpatialState& state) = default;

  /**
   * @brief Swap the values of the two SpatialState.
   * @param state1 SpatialState to be swapped with 2
   * @param state2 SpatialState to be swapped with 1
   */
  friend void swap(SpatialState& state1, SpatialState& state2);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator.
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  SpatialState& operator=(const SpatialState& state);

  /**
   * @brief Getter of the reference frame as const reference.
   * @return The name of the reference frame
   */
  const std::string& get_reference_frame() const;

  /**
   * @brief Setter of the reference frame.
   * @param reference_frame The reference frame
   */
  virtual void set_reference_frame(const std::string& reference_frame);

  /**
   * @brief Check if the state is compatible for operations with the state given as argument.
   * @param state The state to check compatibility with
   */
  virtual bool is_compatible(const State& state) const override;

  /**
   * @brief Overload the ostream operator for printing.
   * @param os The ostream to append the string representing the SpatialState to
   * @param state The SpatialState to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const SpatialState& state);
};

inline void swap(SpatialState& state1, SpatialState& state2) {
  swap(static_cast<State&>(state1), static_cast<State&>(state2));
  std::swap(state1.reference_frame_, state2.reference_frame_);
}

inline SpatialState& SpatialState::operator=(const SpatialState& state) {
  SpatialState tmp(state);
  swap(*this, tmp);
  return *this;
}

inline const std::string& SpatialState::get_reference_frame() const {
  return this->reference_frame_;
}

inline void SpatialState::set_reference_frame(const std::string& reference_frame) {
  this->reference_frame_ = reference_frame;
}

inline bool SpatialState::is_compatible(const State& state) const {
  bool compatible = (this->get_name() == state.get_name())
      && (this->reference_frame_ == dynamic_cast<const SpatialState&>(state).reference_frame_);
  return compatible;
}
}
