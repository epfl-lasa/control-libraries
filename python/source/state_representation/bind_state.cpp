#include "state_representation_bindings.h"

#include <state_representation/State.hpp>


void state_type(py::module_& m) {
  py::enum_<StateType>(m, "StateType")
      .value("STATE", StateType::STATE)
      .value("CARTESIANSTATE", StateType::CARTESIANSTATE)
      .value("DUALQUATERNIONSTATE", StateType::DUALQUATERNIONSTATE)
      .value("JOINTSTATE", StateType::JOINTSTATE)
      .value("JACOBIANMATRIX", StateType::JACOBIANMATRIX)
      .value("GEOMETRY_SHAPE", StateType::GEOMETRY_SHAPE)
      .value("GEOMETRY_ELLIPSOID", StateType::GEOMETRY_ELLIPSOID)
      .value("PARAMETER_INT", StateType::PARAMETER_INT)
      .value("PARAMETER_INT_ARRAY", StateType::PARAMETER_INT_ARRAY)
      .value("PARAMETER_DOUBLE", StateType::PARAMETER_DOUBLE)
      .value("PARAMETER_DOUBLE_ARRAY", StateType::PARAMETER_DOUBLE_ARRAY)
      .value("PARAMETER_BOOL", StateType::PARAMETER_BOOL)
      .value("PARAMETER_BOOL_ARRAY", StateType::PARAMETER_BOOL_ARRAY)
      .value("PARAMETER_STRING", StateType::PARAMETER_STRING)
      .value("PARAMETER_STRING_ARRAY", StateType::PARAMETER_STRING_ARRAY)
      .value("PARAMETER_CARTESIANSTATE", StateType::PARAMETER_CARTESIANSTATE)
      .value("PARAMETER_CARTESIANPOSE", StateType::PARAMETER_CARTESIANPOSE)
      .value("PARAMETER_JOINTSTATE", StateType::PARAMETER_JOINTSTATE)
      .value("PARAMETER_JOINTPOSITIONS", StateType::PARAMETER_JOINTPOSITIONS)
      .value("PARAMETER_ELLIPSOID", StateType::PARAMETER_ELLIPSOID)
      .value("PARAMETER_MATRIX", StateType::PARAMETER_MATRIX)
      .value("PARAMETER_VECTOR", StateType::PARAMETER_VECTOR)
      .export_values();
}

void state(py::module_& m) {
  py::class_<State> c(m, "State");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const StateType&>(), "Constructor only specifying the type of the state from the StateType enumeration", "type"_a);
  c.def(py::init<const StateType&, const std::string&, const bool&>(), "Constructor with name specification", "type"_a, "name"_a, "empty"_a=true);
  c.def(py::init<const State&>(), "Copy constructor from another State", "state"_a);

  c.def("get_type", &State::get_type, "Getter of the type attribute");
  c.def("is_empty", &State::is_empty, "Getter of the empty attribute");
  c.def("set_empty", &State::set_empty, "Setter of the empty attribute", "empty"_a=true);
  c.def("set_filled", &State::set_filled, "Setter of the empty attribute to false and also reset the timestamp");
  c.def("get_timestamp", &State::get_timestamp, "Getter of the timestamp attribute");
  c.def("set_timestamp", &State::set_timestamp, "Setter of the timestamp attribute");
  c.def("reset_timestamp", &State::reset_timestamp, "Reset the timestamp attribute to now");
  c.def("get_name", &State::get_name, "Getter of the name");
  c.def("set_name", &State::set_name, "Setter of the name");

  c.def("is_deprecated", &State::is_deprecated<std::micro>, "Check if the state is deprecated given a certain time delay with microsecond precision");

  c.def("is_compatible", &State::is_compatible, "Check if the state is compatible for operations with the state given as argument", "state"_a);
  c.def("initialize", &State::initialize, "Initialize the State to a zero value");

  c.def("__copy__", [](const State &state) {
    return State(state);
  });
  c.def("__deepcopy__", [](const State &state, py::dict) {
    return State(state);
  }, "memo"_a);
  c.def("__repr__", [](const State& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
}

void bind_state(py::module_& m) {
  state_type(m);
  state(m);
}