#include "state_representation/State.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

namespace state_representation {
State::State() : type_(StateType::STATE), name_(""), empty_(true) {}

State::State(const StateType& type) : type_(type), name_(""), empty_(true) {}

State::State(const StateType& type, const std::string& name, const bool& empty) :
    type_(type), name_(name), empty_(empty), timestamp_(std::chrono::steady_clock::now()) {}

State::State(const State& state) :
    std::enable_shared_from_this<State>(state),
    type_(state.type_),
    name_(state.name_),
    empty_(state.empty_),
    timestamp_(std::chrono::steady_clock::now()) {}

const StateType& State::get_type() const {
  return this->type_;
}

bool State::is_empty() const {
  return this->empty_;
}

void State::set_empty(bool empty) {
  this->empty_ = empty;
}

void State::set_filled() {
  this->empty_ = false;
  this->reset_timestamp();
}

const std::chrono::time_point<std::chrono::steady_clock>& State::get_timestamp() const {
  return this->timestamp_;
}

void State::set_timestamp(const std::chrono::time_point<std::chrono::steady_clock>& timepoint) {
  this->timestamp_ = timepoint;
}

void State::reset_timestamp() {
  this->timestamp_ = std::chrono::steady_clock::now();
}

const std::string& State::get_name() const {
  return this->name_;
}

void State::set_name(const std::string& name) {
  this->name_ = name;
}

bool State::is_compatible(const State& state) const {
  bool compatible = (this->name_ == state.name_);
  return compatible;
}

void State::initialize() {
  this->empty_ = true;
}

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
  os << " State: " << state.get_name();
  return os;
}

void State::set_type(const StateType& type) {
  this->type_ = type;
}

}// namespace state_representation
