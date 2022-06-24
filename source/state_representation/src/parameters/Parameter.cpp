#include "state_representation/parameters/Parameter.hpp"

#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"

namespace state_representation {

template<>
Parameter<int>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::INT), value_() {}

template<>
Parameter<int>::Parameter(const std::string& name, const int& value) :
    ParameterInterface(name, ParameterType::INT), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<int>>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::INT_ARRAY), value_() {}

template<>
Parameter<std::vector<int>>::Parameter(const std::string& name, const std::vector<int>& value) :
    ParameterInterface(name, ParameterType::INT_ARRAY), value_(value) {
  this->set_filled();
}

template<>
Parameter<double>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::DOUBLE), value_() {}

template<>
Parameter<double>::Parameter(const std::string& name, const double& value) :
    ParameterInterface(name, ParameterType::DOUBLE), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<double>>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::DOUBLE_ARRAY), value_() {}

template<>
Parameter<std::vector<double>>::Parameter(const std::string& name, const std::vector<double>& value) :
    ParameterInterface(name, ParameterType::DOUBLE_ARRAY), value_(value) {
  this->set_filled();
}

template<>
Parameter<bool>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::BOOL), value_() {}

template<>
Parameter<bool>::Parameter(const std::string& name, const bool& value) :
    ParameterInterface(name, ParameterType::BOOL), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<bool>>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::BOOL_ARRAY), value_() {
}

template<>
Parameter<std::vector<bool>>::Parameter(const std::string& name, const std::vector<bool>& value) :
    ParameterInterface(name, ParameterType::BOOL_ARRAY), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::string>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STRING), value_() {}

template<>
Parameter<std::string>::Parameter(const std::string& name, const std::string& value) :
    ParameterInterface(name, ParameterType::STRING), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<std::string>>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STRING_ARRAY), value_() {}

template<>
Parameter<std::vector<std::string>>::Parameter(const std::string& name, const std::vector<std::string>& value) :
    ParameterInterface(name, ParameterType::STRING_ARRAY), value_(value) {
  this->set_filled();
}

template<>
Parameter<CartesianState>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STATE, StateType::CARTESIAN_STATE), value_() {}

template<>
Parameter<CartesianState>::Parameter(const std::string& name, const CartesianState& value) :
    ParameterInterface(name, ParameterType::STATE, StateType::CARTESIAN_STATE), value_(value) {
  this->set_filled();
}

template<>
Parameter<CartesianPose>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STATE, StateType::CARTESIAN_POSE), value_() {}

template<>
Parameter<CartesianPose>::Parameter(const std::string& name, const CartesianPose& value) :
    ParameterInterface(name, ParameterType::STATE, StateType::CARTESIAN_POSE), value_(value) {
  this->set_filled();
}

template<>
Parameter<JointState>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STATE, StateType::JOINT_STATE), value_() {}

template<>
Parameter<JointState>::Parameter(const std::string& name, const JointState& value) :
    ParameterInterface(name, ParameterType::STATE, StateType::JOINT_STATE), value_(value) {
  this->set_filled();
}

template<>
Parameter<JointPositions>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STATE, StateType::JOINT_POSITIONS), value_() {}

template<>
Parameter<JointPositions>::Parameter(const std::string& name, const JointPositions& value) :
    ParameterInterface(name, ParameterType::STATE, StateType::JOINT_POSITIONS), value_(value) {
  this->set_filled();
}

template<>
Parameter<Ellipsoid>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::STATE, StateType::GEOMETRY_ELLIPSOID), value_() {
}

template<>
Parameter<Ellipsoid>::Parameter(const std::string& name, const Ellipsoid& value) :
    ParameterInterface(name, ParameterType::STATE, StateType::GEOMETRY_ELLIPSOID), value_(value) {
  this->set_filled();
}

template<>
Parameter<Eigen::MatrixXd>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::MATRIX), value_() {}

template<>
Parameter<Eigen::MatrixXd>::Parameter(const std::string& name, const Eigen::MatrixXd& value) :
    ParameterInterface(name, ParameterType::MATRIX), value_(value) {
  this->set_filled();
}

template<>
Parameter<Eigen::VectorXd>::Parameter(const std::string& name) :
    ParameterInterface(name, ParameterType::VECTOR), value_() {}

template<>
Parameter<Eigen::VectorXd>::Parameter(const std::string& name, const Eigen::VectorXd& value) :
    ParameterInterface(name, ParameterType::VECTOR), value_(value) {
  this->set_filled();
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Parameter<T>& parameter) {
  if (parameter.is_empty()) {
    os << "Parameter " << parameter.get_name() << " is empty";
  } else {
    os << "Parameter " << parameter.get_name() << ": " << parameter.get_value();
  }
  return os;
}

template std::ostream& operator<<(std::ostream& os, const Parameter<int>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<double>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<bool>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<std::string>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<CartesianState>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<CartesianPose>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<JointState>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<JointPositions>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<Ellipsoid>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<Eigen::MatrixXd>& parameter);
template std::ostream& operator<<(std::ostream& os, const Parameter<Eigen::VectorXd>& parameter);

template<>
std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<int>>& parameter) {
  if (parameter.is_empty()) {
    os << "Parameter " << parameter.get_name() << " is empty" << std::endl;
  } else {
    os << "Parameter " << parameter.get_name() << ": ";
    for (auto& v: parameter.get_value()) {
      os << v << " | ";
    }
    os << std::endl;
  }
  return os;
}

template<>
std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<double>>& parameter) {
  if (parameter.is_empty()) {
    os << "Parameter " << parameter.get_name() << " is empty" << std::endl;
  } else {
    os << "Parameter " << parameter.get_name() << ": ";
    for (auto& v: parameter.get_value()) {
      os << v << " | ";
    }
    os << std::endl;
  }
  return os;
}

template<>
std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<bool>>& parameter) {
  if (parameter.is_empty()) {
    os << "Parameter " << parameter.get_name() << " is empty" << std::endl;
  } else {
    os << "Parameter " << parameter.get_name() << ": ";
    for (auto v: parameter.get_value()) {
      os << v << " | ";
    }
    os << std::endl;
  }
  return os;
}

template<>
std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<std::string>>& parameter) {
  if (parameter.is_empty()) {
    os << "Parameter " << parameter.get_name() << " is empty" << std::endl;
  } else {
    os << "Parameter " << parameter.get_name() << ": ";
    for (auto& v: parameter.get_value()) {
      os << v << " | ";
    }
    os << std::endl;
  }
  return os;
}
}// namespace state_representation
