#include "state_representation/State.hpp"

namespace state_representation {
State::State() : type_(StateType::STATE), name_("none"), empty_(true) {}

State::State(const StateType& type) : type_(type), name_("none"), empty_(true) {}

State::State(const StateType& type, const std::string& name, const bool& empty) : type_(type), name_(name), empty_(empty), timestamp_(std::chrono::steady_clock::now()) {}

State::State(const State& state) : type_(state.type_), name_(state.name_), empty_(state.empty_), timestamp_(std::chrono::steady_clock::now()) {}

std::ostream& operator<<(std::ostream& os, const State& state) {
  if (state.is_empty()) {
    os << "Empty ";
  }
  os << " State: " << state.get_name() << std::endl;
  return os;
}
}// namespace state_representation