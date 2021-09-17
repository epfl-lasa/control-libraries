#include "parameter_container.h"

ParameterContainer::ParameterContainer(const std::string& name, const StateType& type) :
    ParameterInterface(type, name) {}

ParameterContainer::ParameterContainer(const std::string& name, const py::object& value, const StateType& type) :
    ParameterInterface(type, name) {
  set_value(value);
}

ParameterContainer::ParameterContainer(const ParameterContainer& parameter) :
    ParameterInterface(parameter.get_type(), parameter.get_name()), values(parameter.values) {}

void ParameterContainer::set_value(const py::object& value) {
  switch (this->get_type()) {
    case StateType::PARAMETER_INT:
      values.int_value = value.cast<int>();
      break;
    case StateType::PARAMETER_INT_ARRAY:
      values.int_array_value = value.cast < std::vector < int >> ();
      break;
    case StateType::PARAMETER_DOUBLE:
      values.double_value = value.cast<double>();
      break;
    case StateType::PARAMETER_DOUBLE_ARRAY:
      values.double_array_value = value.cast < std::vector < double >> ();
      break;
    case StateType::PARAMETER_BOOL:
      values.bool_value = value.cast<bool>();
      break;
    case StateType::PARAMETER_BOOL_ARRAY:
      values.bool_array_value = value.cast < std::vector < bool >> ();
      break;
    case StateType::PARAMETER_STRING:
      values.string_value = value.cast<std::string>();
      break;
    case StateType::PARAMETER_STRING_ARRAY:
      values.string_array_value = value.cast < std::vector < std::string >> ();
      break;
    case StateType::PARAMETER_CARTESIANSTATE:
      values.cartesian_state = value.cast<CartesianState>();
      break;
    case StateType::PARAMETER_CARTESIANPOSE:
      values.cartesian_pose = value.cast<CartesianPose>();
      break;
    case StateType::PARAMETER_JOINTSTATE:
      values.joint_state = value.cast<JointState>();
      break;
    case StateType::PARAMETER_JOINTPOSITIONS:
      values.joint_positions = value.cast<JointPositions>();
      break;
    case StateType::PARAMETER_MATRIX:
      values.matrix_value = value.cast<Eigen::MatrixXd>();
      break;
    case StateType::PARAMETER_VECTOR:
      values.vector_value = value.cast<Eigen::VectorXd>();
      break;
    default:
      throw std::invalid_argument("This StateType is not a valid Parameter.");
      break;
  }
}

py::object ParameterContainer::get_value() {
  switch (this->get_type()) {
    case StateType::PARAMETER_INT:
      return py::cast(values.int_value);
    case StateType::PARAMETER_INT_ARRAY:
      return py::cast(values.int_array_value);
    case StateType::PARAMETER_DOUBLE:
      return py::cast(values.double_value);
    case StateType::PARAMETER_DOUBLE_ARRAY:
      return py::cast(values.double_array_value);
    case StateType::PARAMETER_BOOL:
      return py::cast(values.bool_value);
    case StateType::PARAMETER_BOOL_ARRAY:
      return py::cast(values.bool_array_value);
    case StateType::PARAMETER_STRING:
      return py::cast(values.string_value);
    case StateType::PARAMETER_STRING_ARRAY:
      return py::cast(values.string_array_value);
    case StateType::PARAMETER_CARTESIANSTATE:
      return py::cast(values.cartesian_state);
    case StateType::PARAMETER_CARTESIANPOSE:
      return py::cast(values.cartesian_pose);
    case StateType::PARAMETER_JOINTSTATE:
      return py::cast(values.joint_state);
    case StateType::PARAMETER_JOINTPOSITIONS:
      return py::cast(values.joint_positions);
    case StateType::PARAMETER_MATRIX:
      return py::cast(values.matrix_value);
    case StateType::PARAMETER_VECTOR:
      return py::cast(values.vector_value);
    default:
      return py::none();
  }
}