#include "state_representation_bindings.h"
#include "parameter_container.h"

void parameter_interface(py::module_& m) {
  py::class_<ParameterInterface, State> c(m, "ParameterInterface");

  c.def(py::init<const StateType&, const std::string&>(), "Constructor with parameter name and type of the parameter", "type"_a, "name"_a);
  c.def(py::init<const ParameterInterface&>(), "Copy constructor from another ParameterInterface", "parameter"_a);
}

void parameter(py::module_& m) {
  py::class_<ParameterContainer, ParameterInterface> c(m, "Parameter");

  c.def(py::init<const std::string&, const StateType&>(), "Constructor of a parameter with name and type", "name"_a, "type"_a);
  c.def(py::init<const std::string&, const py::object&, const StateType&>(), "Constructor of a parameter with name, value and type", "name"_a, "value"_a, "type"_a);
  c.def(py::init<const ParameterContainer&>(), "Copy constructor from another Parameter", "parameter"_a);

  c.def("get_value", &ParameterContainer::get_value, "Getter of the value attribute.");
  c.def("set_value", &ParameterContainer::set_value, "Setter of the value attribute.", py::arg("value"));

  c.def("__repr__", [](const ParameterContainer& parameter) {
    std::stringstream buffer;
    switch (parameter.get_type()) {
      case StateType::PARAMETER_INT: {
        Parameter<int> param(parameter.get_name(), parameter.values.int_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_INT_ARRAY: {
        Parameter<std::vector<int>> param(parameter.get_name(), parameter.values.int_array_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_DOUBLE: {
        Parameter<double> param(parameter.get_name(), parameter.values.double_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_DOUBLE_ARRAY: {
        Parameter<std::vector<double>> param(parameter.get_name(), parameter.values.double_array_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_BOOL: {
        Parameter<bool> param(parameter.get_name(), parameter.values.bool_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_BOOL_ARRAY: {
        Parameter<std::vector<bool>> param(parameter.get_name(), parameter.values.bool_array_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_STRING: {
        Parameter<std::string> param(parameter.get_name(), parameter.values.string_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_STRING_ARRAY: {
        Parameter<std::vector<std::string>> param(parameter.get_name(), parameter.values.string_array_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_CARTESIANSTATE: {
        Parameter<CartesianState> param(parameter.get_name(), parameter.values.cartesian_state);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_CARTESIANPOSE: {
        Parameter<CartesianPose> param(parameter.get_name(), parameter.values.cartesian_pose);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_JOINTSTATE: {
        Parameter<JointState> param(parameter.get_name(), parameter.values.joint_state);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_JOINTPOSITIONS: {
        Parameter<JointPositions> param(parameter.get_name(), parameter.values.joint_positions);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_MATRIX: {
        Parameter<Eigen::MatrixXd> param(parameter.get_name(), parameter.values.matrix_value);
        buffer << param;
        break;
      }
      case StateType::PARAMETER_VECTOR: {
        Parameter<Eigen::VectorXd> param(parameter.get_name(), parameter.values.vector_value);
        buffer << param;
        break;
      }
      default:
        break;
    }
    return buffer.str();
  });
}

void bind_parameters(py::module_& m) {
  parameter_interface(m);
  parameter(m);
}