#include "clproto_bindings.h"
#include "parameter_container.h"

#include <string>

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/JointPositions.hpp>
#include <state_representation/robot/JointVelocities.hpp>
#include <state_representation/robot/JointAccelerations.hpp>
#include <state_representation/robot/JointTorques.hpp>

using namespace clproto;
using namespace state_representation;

py::bytes encode_parameter_container(const ParameterContainer& container) {
  switch (container.get_type()) {
    case StateType::PARAMETER_INT:
      return py::bytes(encode(Parameter(container.get_name(), container.values.int_value)));
    case StateType::PARAMETER_INT_ARRAY:
      return py::bytes(encode(Parameter(container.get_name(), container.values.int_array_value)));
    case StateType::PARAMETER_DOUBLE:
      return py::bytes(encode(Parameter(container.get_name(), container.values.double_value)));
    case StateType::PARAMETER_DOUBLE_ARRAY:
      return py::bytes(encode(Parameter(container.get_name(), container.values.double_array_value)));
    case StateType::PARAMETER_BOOL:
      return py::bytes(encode(Parameter(container.get_name(), container.values.bool_value)));
    case StateType::PARAMETER_BOOL_ARRAY:
      return py::bytes(encode(Parameter(container.get_name(), container.values.bool_array_value)));
    case StateType::PARAMETER_STRING:
      return py::bytes(encode(Parameter(container.get_name(), container.values.string_value)));
    case StateType::PARAMETER_STRING_ARRAY:
      return py::bytes(encode(Parameter(container.get_name(), container.values.string_array_value)));
    case StateType::PARAMETER_MATRIX:
      return py::bytes(encode(Parameter(container.get_name(), container.values.matrix_value)));
    case StateType::PARAMETER_VECTOR:
      return py::bytes(encode(Parameter(container.get_name(), container.values.vector_value)));
    default:
      throw std::invalid_argument("This StateType is not a valid Parameter.");
      break;
  }
}

py::object decode_parameter_container(const std::string& msg) {
  switch (check_parameter_message_type(msg)) {
    case ParameterMessageType::INT:
      return py::cast(decode<Parameter<int>>(msg));
    case ParameterMessageType::INT_ARRAY:
      return py::cast(decode<Parameter<std::vector<int>>>(msg));
    case ParameterMessageType::DOUBLE:
      return py::cast(decode<Parameter<double>>(msg));
    case ParameterMessageType::DOUBLE_ARRAY:
      return py::cast(decode<Parameter<std::vector<double>>>(msg));
    case ParameterMessageType::BOOL:
      return py::cast(decode<Parameter<bool>>(msg));
    case ParameterMessageType::BOOL_ARRAY:
      return py::cast(decode<Parameter<std::vector<bool>>>(msg));
    case ParameterMessageType::STRING:
      return py::cast(decode<Parameter<std::string>>(msg));
    case ParameterMessageType::STRING_ARRAY:
      return py::cast(decode<Parameter<std::vector<std::string>>>(msg));
    case ParameterMessageType::MATRIX:
      return py::cast(decode<Parameter<Eigen::MatrixXd>>(msg));
    case ParameterMessageType::VECTOR:
      return py::cast(decode<Parameter<Eigen::VectorXd>>(msg));
    default:
      throw std::invalid_argument("The message is not a valid encoded Parameter.");
      break;
  }
}

void parameter_message_type(py::module_& m) {
  py::enum_<ParameterMessageType>(m, "ParameterMessageType")
      .value("UNKNOWN_PARAMETER", ParameterMessageType::UNKNOWN_PARAMETER)
      .value("INT", ParameterMessageType::INT)
      .value("INT_ARRAY", ParameterMessageType::INT_ARRAY)
      .value("DOUBLE", ParameterMessageType::DOUBLE)
      .value("DOUBLE_ARRAY", ParameterMessageType::DOUBLE_ARRAY)
      .value("BOOL", ParameterMessageType::BOOL)
      .value("BOOL_ARRAY", ParameterMessageType::BOOL_ARRAY)
      .value("STRING", ParameterMessageType::STRING)
      .value("STRING_ARRAY", ParameterMessageType::STRING_ARRAY)
      .value("MATRIX", ParameterMessageType::MATRIX)
      .value("VECTOR", ParameterMessageType::VECTOR)
      .export_values();
}

void bind_clproto_parameters(py::module_& m) {
  parameter_message_type(m);
}