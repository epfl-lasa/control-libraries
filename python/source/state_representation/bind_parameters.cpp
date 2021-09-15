#include "state_representation_bindings.h"

#include <state_representation/State.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/robot/JointPositions.hpp>

struct ParameterValues {
  double double_value;
  std::vector<double> double_array_value;
  bool bool_value;
  std::vector<bool> bool_array_value;
  std::string string_value;
  std::vector<std::string> string_array_value;
  CartesianState cartesian_state;
  CartesianPose cartesian_pose;
  JointState joint_state;
  JointPositions joint_positions;
  Eigen::MatrixXd matrix_value;
  Eigen::VectorXd vector_value;
};

class ParameterContainer : public ParameterInterface {
public:
  ParameterContainer(const std::string& name, const StateType& type) : ParameterInterface(type, name) {}

  explicit ParameterContainer(const ParameterContainer& parameter) : ParameterInterface(parameter.get_type(), parameter.get_name()) {}

  ParameterValues values;
};

void parameter_interface(py::module_& m) {
  py::class_<ParameterInterface, State> c(m, "ParameterInterface");

  c.def(py::init<const StateType&, const std::string&>(), "Constructor with parameter name and type of the parameter", "type"_a, "name"_a);
  c.def(py::init<const ParameterInterface&>(), "Copy constructor from another ParameterInterface", "parameter"_a);
}

void parameter(py::module_& m) {
  py::class_<ParameterContainer, ParameterInterface> c(m, "Parameter");

  c.def(py::init<const std::string&, const StateType&>(), "Constructor of a parameter with name and type", "name"_a, "type"_a);
  c.def(py::init<const ParameterContainer&>(), "Copy constructor from another Parameter", "parameter"_a);

  c.def("set_value", [](ParameterContainer& parameter, const py::object& value) {
    switch (parameter.get_type()) {
      case StateType::PARAMETER_DOUBLE:
        parameter.values.double_value = value.cast<double>();
        break;
      case StateType::PARAMETER_DOUBLE_ARRAY:
        parameter.values.double_array_value = value.cast<std::vector<double>>();
        break;
      case StateType::PARAMETER_BOOL:
        parameter.values.bool_value = value.cast<bool>();
        break;
      case StateType::PARAMETER_BOOL_ARRAY:
        parameter.values.bool_array_value = value.cast<std::vector<bool>>();
        break;
      case StateType::PARAMETER_STRING:
        parameter.values.string_value = value.cast<std::string>();
        break;
      case StateType::PARAMETER_STRING_ARRAY:
        parameter.values.string_array_value = value.cast<std::vector<std::string>>();
        break;
      case StateType::PARAMETER_CARTESIANSTATE:
        parameter.values.cartesian_state = value.cast<CartesianState>();
        break;
      case StateType::PARAMETER_CARTESIANPOSE:
        parameter.values.cartesian_pose = value.cast<CartesianPose>();
        break;
      case StateType::PARAMETER_JOINTSTATE:
        parameter.values.joint_state = value.cast<JointState>();
        break;
      case StateType::PARAMETER_JOINTPOSITIONS:
        parameter.values.joint_positions = value.cast<JointPositions>();
        break;
      case StateType::PARAMETER_MATRIX:
        parameter.values.matrix_value = value.cast<Eigen::MatrixXd>();
        break;
      case StateType::PARAMETER_VECTOR:
        parameter.values.vector_value = value.cast<Eigen::VectorXd>();
        break;
      default:
        break;
    }
  }, "Setter of the value attribute.", py::arg("value"));

  c.def("get_value", [](const ParameterContainer& parameter) -> py::object {
    switch (parameter.get_type()) {
      case StateType::PARAMETER_DOUBLE:
        return py::cast(parameter.values.double_value);
      case StateType::PARAMETER_DOUBLE_ARRAY:
        return py::cast(parameter.values.double_array_value);
      case StateType::PARAMETER_BOOL:
        return py::cast(parameter.values.bool_value);
      case StateType::PARAMETER_BOOL_ARRAY:
        return py::cast(parameter.values.bool_array_value);
      case StateType::PARAMETER_STRING:
        return py::cast(parameter.values.string_value);
      case StateType::PARAMETER_STRING_ARRAY:
        return py::cast(parameter.values.string_array_value);
      case StateType::PARAMETER_CARTESIANSTATE:
        return py::cast(parameter.values.cartesian_state);
      case StateType::PARAMETER_CARTESIANPOSE:
        return py::cast(parameter.values.cartesian_pose);
      case StateType::PARAMETER_JOINTSTATE:
        return py::cast(parameter.values.joint_state);
      case StateType::PARAMETER_JOINTPOSITIONS:
        return py::cast(parameter.values.joint_positions);
      case StateType::PARAMETER_MATRIX:
        return py::cast(parameter.values.matrix_value);
      case StateType::PARAMETER_VECTOR:
        return py::cast(parameter.values.vector_value);
      default:
        return py::none();
    }
  }, "Getter of the value attribute.");

  c.def("__repr__", [](const ParameterContainer& parameter) {
    std::stringstream buffer;
    switch (parameter.get_type()) {
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