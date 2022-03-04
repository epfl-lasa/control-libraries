#include "parameter_container.h"

namespace py_parameter {

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
      values.int_array_value = value.cast<std::vector<int>>();
      break;
    case StateType::PARAMETER_DOUBLE:
      values.double_value = value.cast<double>();
      break;
    case StateType::PARAMETER_DOUBLE_ARRAY:
      values.double_array_value = value.cast<std::vector<double>>();
      break;
    case StateType::PARAMETER_BOOL:
      values.bool_value = value.cast<bool>();
      break;
    case StateType::PARAMETER_BOOL_ARRAY:
      values.bool_array_value = value.cast<std::vector<bool>>();
      break;
    case StateType::PARAMETER_STRING:
      values.string_value = value.cast<std::string>();
      break;
    case StateType::PARAMETER_STRING_ARRAY:
      values.string_array_value = value.cast<std::vector<std::string>>();
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
    case StateType::PARAMETER_ELLIPSOID:
      values.ellipsoid = value.cast<Ellipsoid>();
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
    case StateType::PARAMETER_ELLIPSOID:
      return py::cast(values.ellipsoid);
    case StateType::PARAMETER_MATRIX:
      return py::cast(values.matrix_value);
    case StateType::PARAMETER_VECTOR:
      return py::cast(values.vector_value);
    default:
      return py::none();
  }
}

ParameterContainer interface_ptr_to_container(const std::shared_ptr<ParameterInterface>& parameter) {
  switch (parameter->get_type()) {
    case StateType::PARAMETER_INT:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<int>()), parameter->get_type());
      break;
    case StateType::PARAMETER_INT_ARRAY:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<std::vector<int>>()), parameter->get_type());
      break;
    case StateType::PARAMETER_DOUBLE:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<double>()), parameter->get_type());
      break;
    case StateType::PARAMETER_DOUBLE_ARRAY:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<std::vector<double>>()), parameter->get_type());
      break;
    case StateType::PARAMETER_BOOL:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<bool>()), parameter->get_type());
      break;
    case StateType::PARAMETER_BOOL_ARRAY:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<std::vector<bool>>()), parameter->get_type());
      break;
    case StateType::PARAMETER_STRING:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<std::string>()), parameter->get_type());
      break;
    case StateType::PARAMETER_STRING_ARRAY:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<std::vector<std::string>>()), parameter->get_type());
      break;
    case StateType::PARAMETER_CARTESIANSTATE:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<CartesianState>()), parameter->get_type());
      break;
    case StateType::PARAMETER_CARTESIANPOSE:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<CartesianPose>()), parameter->get_type());
      break;
    case StateType::PARAMETER_JOINTSTATE:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<JointState>()), parameter->get_type());
      break;
    case StateType::PARAMETER_JOINTPOSITIONS:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<JointPositions>()), parameter->get_type());
      break;
    case StateType::PARAMETER_ELLIPSOID:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<Ellipsoid>()), parameter->get_type());
      break;
    case StateType::PARAMETER_MATRIX:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<Eigen::MatrixXd>()), parameter->get_type());
      break;
    case StateType::PARAMETER_VECTOR:
      return ParameterContainer(parameter->get_name(), py::cast(parameter->get_parameter_value<Eigen::VectorXd>()), parameter->get_type());
      break;
    default:
      throw std::invalid_argument("The conversion from this StateType to ParameterContainer is not supported yet.");
      break;
  }
}

std::shared_ptr<ParameterInterface> container_to_interface_ptr(const ParameterContainer& parameter) {
  switch (parameter.get_type()) {
    case StateType::PARAMETER_INT: {
      return make_shared_parameter(parameter.get_name(), parameter.values.int_value);
      break;
    }
    case StateType::PARAMETER_INT_ARRAY: {
      return make_shared_parameter(parameter.get_name(), parameter.values.int_array_value);
      break;
    }
    case StateType::PARAMETER_DOUBLE: {
      return make_shared_parameter(parameter.get_name(), parameter.values.double_value);
      break;
    }
    case StateType::PARAMETER_DOUBLE_ARRAY: {
      return make_shared_parameter(parameter.get_name(), parameter.values.double_array_value);
      break;
    }
    case StateType::PARAMETER_BOOL: {
      return make_shared_parameter(parameter.get_name(), parameter.values.bool_value);
      break;
    }
    case StateType::PARAMETER_BOOL_ARRAY: {
      return make_shared_parameter(parameter.get_name(), parameter.values.bool_array_value);
      break;
    }
    case StateType::PARAMETER_STRING: {
      return make_shared_parameter(parameter.get_name(), parameter.values.string_value);
      break;
    }
    case StateType::PARAMETER_STRING_ARRAY: {
      return make_shared_parameter(parameter.get_name(), parameter.values.string_array_value);
      break;
    }
    case StateType::PARAMETER_CARTESIANSTATE: {
      return make_shared_parameter(parameter.get_name(), parameter.values.cartesian_state);
      break;
    }
    case StateType::PARAMETER_CARTESIANPOSE: {
      return make_shared_parameter(parameter.get_name(), parameter.values.cartesian_pose);
      break;
    }
    case StateType::PARAMETER_JOINTSTATE: {
      return make_shared_parameter(parameter.get_name(), parameter.values.joint_state);
      break;
    }
    case StateType::PARAMETER_JOINTPOSITIONS: {
      return make_shared_parameter(parameter.get_name(), parameter.values.joint_positions);
      break;
    }
    case StateType::PARAMETER_ELLIPSOID: {
      return make_shared_parameter(parameter.get_name(), parameter.values.ellipsoid);
      break;
    }
    case StateType::PARAMETER_MATRIX: {
      return make_shared_parameter(parameter.get_name(), parameter.values.matrix_value);
      break;
    }
      case StateType::PARAMETER_VECTOR: {
        return make_shared_parameter(parameter.get_name(), parameter.values.vector_value);
        break;
      }
    default:
      return {};
      break;
  }
}

std::map<std::string, ParameterContainer>
interface_ptr_to_container_map(const std::map<std::string, std::shared_ptr<ParameterInterface>>& parameters) {
  std::map<std::string, ParameterContainer> parameter_list;
  for (const auto& param_it: parameters) {
    parameter_list.insert(
        std::pair<std::string, ParameterContainer>(param_it.first, interface_ptr_to_container(param_it.second)));
  }
  return parameter_list;
}

std::map<std::string, std::shared_ptr<ParameterInterface>>
container_to_interface_ptr_map(const std::map<std::string, ParameterContainer>& parameters) {
  std::map<std::string, std::shared_ptr<ParameterInterface>> parameter_list;
  for (const auto& param_it: parameters) {
    parameter_list.insert(std::pair<std::string, std::shared_ptr<ParameterInterface>>(param_it.first, container_to_interface_ptr(param_it.second)));
  }
  return parameter_list;
}

std::list<ParameterContainer>
interface_ptr_to_container_list(const std::list<std::shared_ptr<ParameterInterface>>& parameters) {
  std::list<ParameterContainer> parameter_list;
  for (const auto& param_it: parameters) {
    parameter_list.emplace_back(interface_ptr_to_container(param_it));
  }
  return parameter_list;
}

std::list<std::shared_ptr<ParameterInterface>>
container_to_interface_ptr_list(const std::list<ParameterContainer>& parameters) {
  std::list<std::shared_ptr<ParameterInterface>> parameter_list;
  for (const auto& param_it: parameters) {
    parameter_list.emplace_back(container_to_interface_ptr(param_it));
  }
  return parameter_list;
}
}// namespace py_parameter
