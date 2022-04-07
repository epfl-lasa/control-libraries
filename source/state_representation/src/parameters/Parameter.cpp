#include "state_representation/parameters/Parameter.hpp"

#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"

namespace state_representation {

template<>
Parameter<int>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::INT, name) {}

template<>
Parameter<int>::Parameter(const std::string& name, const int& value) :
    ParameterInterface(ParameterType::INT, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<int>>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::INT_ARRAY, name) {}

template<>
Parameter<std::vector<int>>::Parameter(const std::string& name, const std::vector<int>& value) :
    ParameterInterface(ParameterType::INT_ARRAY, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<double>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::DOUBLE, name) {}

template<>
Parameter<double>::Parameter(const std::string& name, const double& value) :
    ParameterInterface(ParameterType::DOUBLE, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<double>>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::DOUBLE_ARRAY, name) {}

template<>
Parameter<std::vector<double>>::Parameter(const std::string& name, const std::vector<double>& value) :
    ParameterInterface(ParameterType::DOUBLE_ARRAY, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<bool>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::BOOL, name) {}

template<>
Parameter<bool>::Parameter(const std::string& name, const bool& value) :
    ParameterInterface(ParameterType::BOOL, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<bool>>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::BOOL_ARRAY, name) {
}

template<>
Parameter<std::vector<bool>>::Parameter(const std::string& name, const std::vector<bool>& value) :
    ParameterInterface(ParameterType::BOOL_ARRAY, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::string>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::STRING, name) {}

template<>
Parameter<std::string>::Parameter(const std::string& name, const std::string& value) :
    ParameterInterface(ParameterType::STRING, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<std::vector<std::string>>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::STRING_ARRAY, name) {}

template<>
Parameter<std::vector<std::string>>::Parameter(const std::string& name, const std::vector<std::string>& value) :
    ParameterInterface(ParameterType::STRING_ARRAY, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<CartesianState>::Parameter(const std::string& name) :
    ParameterInterface(StateType::CARTESIAN_STATE, name) {}

template<>
Parameter<CartesianState>::Parameter(const std::string& name, const CartesianState& value) :
    ParameterInterface(StateType::CARTESIAN_STATE, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<CartesianPose>::Parameter(const std::string& name) :
    ParameterInterface(StateType::CARTESIAN_POSE, name) {}

template<>
Parameter<CartesianPose>::Parameter(const std::string& name, const CartesianPose& value) :
    ParameterInterface(StateType::CARTESIAN_POSE, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<JointState>::Parameter(const std::string& name) :
    ParameterInterface(StateType::JOINT_STATE, name) {}

template<>
Parameter<JointState>::Parameter(const std::string& name, const JointState& value) :
    ParameterInterface(StateType::JOINT_STATE, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<JointPositions>::Parameter(const std::string& name) :
    ParameterInterface(StateType::JOINT_POSITIONS, name) {}

template<>
Parameter<JointPositions>::Parameter(const std::string& name, const JointPositions& value) :
    ParameterInterface(StateType::JOINT_POSITIONS, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<Ellipsoid>::Parameter(const std::string& name, const Ellipsoid& value) :
    ParameterInterface(StateType::GEOMETRY_ELLIPSOID, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<Eigen::MatrixXd>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::MATRIX, name) {}

template<>
Parameter<Eigen::MatrixXd>::Parameter(const std::string& name, const Eigen::MatrixXd& value) :
    ParameterInterface(ParameterType::MATRIX, name), value_(value) {
  this->set_filled();
}

template<>
Parameter<Eigen::VectorXd>::Parameter(const std::string& name) :
    ParameterInterface(ParameterType::VECTOR, name) {}

template<>
Parameter<Eigen::VectorXd>::Parameter(const std::string& name, const Eigen::VectorXd& value) :
    ParameterInterface(ParameterType::VECTOR, name), value_(value) {
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
