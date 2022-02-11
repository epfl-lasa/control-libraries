#pragma once

#include "state_representation_bindings.h"

#include <state_representation/State.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/joint/JointPositions.hpp>

struct ParameterValues {
  int int_value;
  std::vector<int> int_array_value;
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
  Ellipsoid ellipsoid;
  Eigen::MatrixXd matrix_value;
  Eigen::VectorXd vector_value;
};

class ParameterContainer : public ParameterInterface {
public:
  ParameterContainer(const std::string& name, const StateType& type);
  ParameterContainer(const std::string& name, const py::object& value, const StateType& type);
  ParameterContainer(const ParameterContainer& parameter);

  void set_value(const py::object& value);

  py::object get_value();

  ParameterValues values;
};

ParameterContainer parameter_interface_ptr_to_container(const std::shared_ptr<ParameterInterface>& parameter);

std::shared_ptr<ParameterInterface> container_to_parameter_interface_ptr(const ParameterContainer& parameter);