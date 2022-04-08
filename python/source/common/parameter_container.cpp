#include "parameter_container.h"

namespace py_parameter {

ParameterContainer::ParameterContainer(
    const std::string& name, const ParameterType& type, const StateType& parameter_state_type
) : ParameterInterface(name, type, parameter_state_type) {}

ParameterContainer::ParameterContainer(
    const std::string& name, const py::object& value, const ParameterType& type, const StateType& parameter_state_type
) : ParameterInterface(name, type, parameter_state_type) {
  set_value(value);
}

ParameterContainer::ParameterContainer(const ParameterContainer& parameter) :
    ParameterInterface(parameter.get_name(), parameter.get_parameter_type(), parameter.get_parameter_state_type()),
    values(parameter.values) {}

void ParameterContainer::set_value(const py::object& value) {
  switch (this->get_parameter_type()) {
    case ParameterType::INT:
      values.int_value = value.cast<int>();
      break;
    case ParameterType::INT_ARRAY:
      values.int_array_value = value.cast<std::vector<int>>();
      break;
    case ParameterType::DOUBLE:
      values.double_value = value.cast<double>();
      break;
    case ParameterType::DOUBLE_ARRAY:
      values.double_array_value = value.cast<std::vector<double>>();
      break;
    case ParameterType::BOOL:
      values.bool_value = value.cast<bool>();
      break;
    case ParameterType::BOOL_ARRAY:
      values.bool_array_value = value.cast<std::vector<bool>>();
      break;
    case ParameterType::STRING:
      values.string_value = value.cast<std::string>();
      break;
    case ParameterType::STRING_ARRAY:
      values.string_array_value = value.cast<std::vector<std::string>>();
      break;
    case ParameterType::STATE:
      // TODO
      break;
    case ParameterType::MATRIX:
      values.matrix_value = value.cast<Eigen::MatrixXd>();
      break;
    case ParameterType::VECTOR:
      values.vector_value = value.cast<Eigen::VectorXd>();
      break;
    default:
      throw std::invalid_argument("The ParameterType is invalid.");
      break;
  }
}

py::object ParameterContainer::get_value() {
  switch (this->get_parameter_type()) {
    case ParameterType::INT:
      return py::cast(values.int_value);
    case ParameterType::INT_ARRAY:
      return py::cast(values.int_array_value);
    case ParameterType::DOUBLE:
      return py::cast(values.double_value);
    case ParameterType::DOUBLE_ARRAY:
      return py::cast(values.double_array_value);
    case ParameterType::BOOL:
      return py::cast(values.bool_value);
    case ParameterType::BOOL_ARRAY:
      return py::cast(values.bool_array_value);
    case ParameterType::STRING:
      return py::cast(values.string_value);
    case ParameterType::STRING_ARRAY:
      return py::cast(values.string_array_value);
    case ParameterType::STATE:
      // TODO
      break;
    case ParameterType::MATRIX:
      return py::cast(values.matrix_value);
    case ParameterType::VECTOR:
      return py::cast(values.vector_value);
    default:
      break;
  }
  return py::none();
}

ParameterContainer interface_ptr_to_container(const std::shared_ptr<ParameterInterface>& parameter) {
  switch (parameter->get_parameter_type()) {
    case ParameterType::INT:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value<int>()), ParameterType::INT);
    case ParameterType::INT_ARRAY:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value < std::vector < int>>()),
          ParameterType::INT_ARRAY);
    case ParameterType::DOUBLE:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value<double>()), ParameterType::DOUBLE);
    case ParameterType::DOUBLE_ARRAY:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value < std::vector < double>>()),
          ParameterType::DOUBLE_ARRAY);
    case ParameterType::BOOL:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value<bool>()), ParameterType::BOOL);
    case ParameterType::BOOL_ARRAY:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value < std::vector < bool>>()),
          ParameterType::BOOL_ARRAY);
    case ParameterType::STRING:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value<std::string>()), ParameterType::STRING);
    case ParameterType::STRING_ARRAY:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value < std::vector < std::string>>()),
          ParameterType::STRING_ARRAY);
    case ParameterType::STATE:
      // TODO
      break;
    case ParameterType::MATRIX:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value<Eigen::MatrixXd>()), ParameterType::MATRIX);
    case ParameterType::VECTOR:
      return ParameterContainer(
          parameter->get_name(), py::cast(parameter->get_parameter_value<Eigen::VectorXd>()), ParameterType::VECTOR);
    default:
      break;
  }
  throw std::invalid_argument("The conversion from this ParameterType to ParameterContainer is not supported.");
}

std::shared_ptr<ParameterInterface> container_to_interface_ptr(const ParameterContainer& parameter) {
  switch (parameter.get_parameter_type()) {
    case ParameterType::INT:
      return make_shared_parameter(parameter.get_name(), parameter.values.int_value);
    case ParameterType::INT_ARRAY:
      return make_shared_parameter(parameter.get_name(), parameter.values.int_array_value);
    case ParameterType::DOUBLE:
      return make_shared_parameter(parameter.get_name(), parameter.values.double_value);
    case ParameterType::DOUBLE_ARRAY:
      return make_shared_parameter(parameter.get_name(), parameter.values.double_array_value);
    case ParameterType::BOOL:
      return make_shared_parameter(parameter.get_name(), parameter.values.bool_value);
    case ParameterType::BOOL_ARRAY:
      return make_shared_parameter(parameter.get_name(), parameter.values.bool_array_value);
    case ParameterType::STRING:
      return make_shared_parameter(parameter.get_name(), parameter.values.string_value);
    case ParameterType::STRING_ARRAY:
      return make_shared_parameter(parameter.get_name(), parameter.values.string_array_value);
    case ParameterType::STATE:
      // TODO
      break;
    case ParameterType::MATRIX:
      return make_shared_parameter(parameter.get_name(), parameter.values.matrix_value);
    case ParameterType::VECTOR:
      return make_shared_parameter(parameter.get_name(), parameter.values.vector_value);
    default:
      break;
  }
  return {};
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
    parameter_list.insert(
        std::pair<std::string, std::shared_ptr<ParameterInterface>>(
            param_it.first, container_to_interface_ptr(param_it.second)
        )
    );
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
