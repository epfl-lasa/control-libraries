#include "state_representation/State.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

namespace state_representation {
State::State() : type_(StateType::STATE), name_(""), empty_(true) {}

State::State(const StateType& type) : type_(type), name_(""), empty_(true) {}

State::State(const StateType& type, const std::string& name, const bool& empty)
    : type_(type), name_(name), empty_(empty), timestamp_(std::chrono::steady_clock::now()) {}

State::State(const State& state)
    : type_(state.type_), name_(state.name_), empty_(state.empty_), timestamp_(std::chrono::steady_clock::now()) {}

void State::set_data(const Eigen::VectorXd&) {
  throw exceptions::NotImplementedException("set_data() is not implemented for the base State class");
}

void State::set_data(const std::vector<double>&) {
  throw exceptions::NotImplementedException("set_data() is not implemented for the base State class");
}

void State::set_data(const Eigen::MatrixXd&) {
  throw exceptions::NotImplementedException("set_data() is not implemented for the base State class");
}

std::ostream& operator<<(std::ostream& os, const State& state) {
  if (state.is_empty()) {
    os << "Empty ";
  }
  os << " State: " << state.get_name() << std::endl;
  return os;
}
}// namespace state_representation
