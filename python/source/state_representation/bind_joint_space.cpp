#include "state_representation_bindings.h"

#include <state_representation/State.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/JointPositions.hpp>
#include <state_representation/robot/JointVelocities.hpp>
#include <state_representation/robot/JointTorques.hpp>

void joint_state_variable(py::module_& m) {
  py::enum_<JointStateVariable>(m, "JointStateVariable")
      .value("POSITIONS", JointStateVariable::POSITIONS)
      .value("VELOCITIES", JointStateVariable::VELOCITIES)
      .value("ACCELERATIONS", JointStateVariable::ACCELERATIONS)
      .value("TORQUES", JointStateVariable::TORQUES)
      .value("ALL", JointStateVariable::ALL)
      .export_values();
}

void joint_state(py::module_& m) {
  m.def("dist", py::overload_cast<const JointState&, const JointState&, const JointStateVariable&>(&state_representation::dist), "Compute the distance between two JointStates", "s1"_a, "s2"_a, "state_variable_type"_a=JointStateVariable::ALL);

  py::class_<JointState, State> c(m, "JointState");
  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const JointState&>(), "Copy constructor of a JointState", "state"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointState::Zero), "Constructor for the zero JointState", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointState::Zero), "Constructor for the zero JointState", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointState::Random), "Constructor for the random JointState", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointState::Random), "Constructor for the random JointState", "robot_name"_a, "joint_names"_a);

  c.def("get_size", &JointState::get_size, "Getter of the size from the attributes");
  c.def("get_names", &JointState::get_names, "Getter of the names attribute");
  c.def("get_positions", &JointState::get_positions, "Getter of the positions attribute");
  c.def("get_velocities", &JointState::get_velocities, "Getter of the velocities attribute");
  c.def("get_accelerations", &JointState::get_accelerations, "Getter of the accelerations attribute");
  c.def("get_torques", &JointState::get_torques, "Getter of the torques attribute");

  c.def("set_names", py::overload_cast<unsigned int>(&JointState::set_names), "Setter of the names attribute from the number of joints");
  c.def("set_names", py::overload_cast<const std::vector<std::string>&>(&JointState::set_names), "Setter of the names attribute");
  c.def("set_positions", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_positions), "Setter of the positions attribute from a vector");
  c.def("set_positions", py::overload_cast<const std::vector<double>&>(&JointState::set_positions), "Setter of the positions attribute from a list");
  c.def("set_velocities", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_velocities), "Setter of the velocities attribute from a vector");
  c.def("set_velocities", py::overload_cast<const std::vector<double>&>(&JointState::set_velocities), "Setter of the velocities attribute from a list");
  c.def("set_accelerations", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_accelerations), "Setter of the accelerations attribute from a vector");
  c.def("set_accelerations", py::overload_cast<const std::vector<double>&>(&JointState::set_accelerations), "Setter of the accelerations attribute from a list");
  c.def("set_torques", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_torques), "Setter of the torques attribute from a vector");
  c.def("set_torques", py::overload_cast<const std::vector<double>&>(&JointState::set_torques), "Setter of the torques attribute from a list");

  c.def("set_zero", &JointState::set_zero, "Set the JointState to a zero value");
  c.def("clamp_state_variable", py::overload_cast<double, const JointStateVariable&, double>(&JointState::clamp_state_variable), "Clamp inplace the magnitude of the a specific state variable (velocities, accelerations or forces)", "value"_a, "state_variable_type"_a, "noise_ratio"_a=double(0));
  c.def("clamp_state_variable", py::overload_cast<const Eigen::ArrayXd&, const JointStateVariable&, const Eigen::ArrayXd&>(&JointState::clamp_state_variable), "Clamp inplace the magnitude of the a specific state variable (velocities, accelerations or forces)", "max_absolute_value_array"_a, "state_variable_type"_a, "noise_ratio_array"_a);
  c.def("copy", &JointState::copy, "Return a copy of the JointState");
  c.def("data", &JointState::data, "Returns the data as the concatenation of all the state variables in a single vector");
  c.def("array", &JointState::array, "Returns the data vector as an array");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_data), "Set the data of the state from all the state variables in a single vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointState::set_data), "Set the data of the state from all the state variables in a single list", "data"_a);

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def(py::self -= py::self);
  c.def(py::self - py::self);
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(py::self *= Eigen::ArrayXd());
  c.def(py::self * Eigen::ArrayXd());
  c.def(py::self *= Eigen::MatrixXd());
  c.def(py::self * Eigen::MatrixXd());
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def(double() * py::self);
  c.def(Eigen::ArrayXd() * py::self);
  c.def(Eigen::MatrixXd() * py::self);

  c.def("dist", &JointState::dist, "Compute the distance to another state as the sum of distances between each features", "state"_a, "state_variable_type"_a=JointStateVariable::ALL);

  c.def("to_list", &JointState::to_std_vector, "Return the state as a list");

  c.def("__repr__", [](const JointState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
}

void joint_positions(py::module_& m) {
  py::class_<JointPositions, JointState> c(m, "JointPositions");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and position values provided", "robot_name"_a, "positions"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and position values provided", "robot_name"_a, "joint_names"_a, "positions"_a);
  c.def(py::init<const JointPositions&>(), "Copy constructor", "positions"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);
  c.def(py::init<const JointVelocities&>(), "Integration constructor from a JointVelocities by considering that it is equivalent to multiplying the velocities by 1 second", "velocities"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointPositions::Zero), "Constructor for the zero JointPositions", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointPositions::Zero), "Constructor for the zero JointPositions", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointPositions::Random), "Constructor for the random JointPositions", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointPositions::Random), "Constructor for the random JointPositions", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "velocities",
      "accelerations",
      "torques",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointPositions&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointPositions& positions) -> JointPositions { return positions; }, "Deleted method from parent class.");
  }

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def(py::self -= py::self);
  c.def(py::self - py::self);
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(py::self *= Eigen::ArrayXd());
  c.def(py::self * Eigen::ArrayXd());
  c.def(py::self *= Eigen::MatrixXd());
  c.def(py::self * Eigen::MatrixXd());
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def(py::self / std::chrono::nanoseconds());
  c.def(double() * py::self);
  c.def(Eigen::ArrayXd() * py::self);
  c.def(Eigen::MatrixXd() * py::self);

  c.def("copy", &JointPositions::copy, "Return a copy of the JointPositions");
  c.def("data", &JointPositions::data, "Returns the positions data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointPositions::set_data), "Set the positions data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointPositions::set_data), "Set the positions data from a list", "data"_a);

  c.def("__repr__", [](const JointPositions& positions) {
    std::stringstream buffer;
    buffer << positions;
    return buffer.str();
  });
}

void joint_velocities(py::module_& m) {
  py::class_<JointVelocities, JointState> c(m, "JointVelocities");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and velocity values provided", "robot_name"_a, "velocities"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and velocity values provided", "robot_name"_a, "joint_names"_a, "velocities"_a);
  c.def(py::init<const JointVelocities&>(), "Copy constructor", "velocities"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);
  c.def(py::init<const JointPositions&>(), "Differentiation constructor from a JointPositions by considering that it is equivalent to dividing the positions by 1 second", "positions"_a);
  c.def(py::init<const JointAccelerations&>(), "Integration constructor from a JointAccelerations by considering that it is equivalent to multiplying the accelerations by 1 second", "accelerations"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointVelocities::Zero), "Constructor for the zero JointVelocities", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointVelocities::Zero), "Constructor for the zero JointVelocities", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointVelocities::Random), "Constructor for the random JointVelocities", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointVelocities::Random), "Constructor for the random JointVelocities", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "positions",
      "accelerations",
      "torques",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointVelocities&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointVelocities& velocities) -> JointVelocities { return velocities; }, "Deleted method from parent class.");
  }

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def(py::self -= py::self);
  c.def(py::self - py::self);
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(py::self *= Eigen::ArrayXd());
  c.def(py::self * Eigen::ArrayXd());
  c.def(py::self *= Eigen::MatrixXd());
  c.def(py::self * Eigen::MatrixXd());
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def(py::self / std::chrono::nanoseconds());
  c.def(py::self * std::chrono::nanoseconds());
  c.def(double() * py::self);
  c.def(Eigen::ArrayXd() * py::self);
  c.def(Eigen::MatrixXd() * py::self);

  c.def("copy", &JointVelocities::copy, "Return a copy of the JointVelocities");
  c.def("data", &JointVelocities::data, "Returns the velocities data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointVelocities::set_data), "Set the velocities data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointVelocities::set_data), "Set the velocities data from a list", "data"_a);

  c.def("clamp", py::overload_cast<double, double>(&JointVelocities::clamp), "Clamp inplace the magnitude of the velocity to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamped", py::overload_cast<double, double>(&JointVelocities::clamp), "Return the velocity clamped to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamp", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointVelocities::clamp), "Clamp inplace the magnitude of the velocity to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);
  c.def("clamped", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointVelocities::clamp), "Return the velocity clamped to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);

  c.def("__repr__", [](const JointVelocities& velocities) {
    std::stringstream buffer;
    buffer << velocities;
    return buffer.str();
  });
}

void joint_accelerations(py::module_& m) {
  py::class_<JointAccelerations, JointState> c(m, "JointAccelerations");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and acceleration values provided", "robot_name"_a, "accelerations"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and acceleration values provided", "robot_name"_a, "joint_names"_a, "accelerations"_a);
  c.def(py::init<const JointAccelerations&>(), "Copy constructor", "accelerations"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);
  c.def(py::init<const JointVelocities&>(), "Differentiation constructor from a JointVelocities by considering that it is equivalent to dividing the velocities by 1 second", "velocities"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointAccelerations::Zero), "Constructor for the zero JointAccelerations", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointAccelerations::Zero), "Constructor for the zero JointAccelerations", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointAccelerations::Random), "Constructor for the random JointAccelerations", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointAccelerations::Random), "Constructor for the random JointAccelerations", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "positions",
      "velocities",
      "torques",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointAccelerations&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointAccelerations& accelerations) -> JointAccelerations { return accelerations; }, "Deleted method from parent class.");
  }

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def(py::self -= py::self);
  c.def(py::self - py::self);
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(py::self *= Eigen::ArrayXd());
  c.def(py::self * Eigen::ArrayXd());
  c.def(py::self *= Eigen::MatrixXd());
  c.def(py::self * Eigen::MatrixXd());
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def(py::self * std::chrono::nanoseconds());
  c.def(double() * py::self);
  c.def(Eigen::ArrayXd() * py::self);
  c.def(Eigen::MatrixXd() * py::self);

  c.def("copy", &JointAccelerations::copy, "Return a copy of the JointAccelerations");
  c.def("data", &JointAccelerations::data, "Returns the accelerations data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointAccelerations::set_data), "Set the accelerations data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointAccelerations::set_data), "Set the accelerations data from a list", "data"_a);

  c.def("clamp", py::overload_cast<double, double>(&JointAccelerations::clamp), "Clamp inplace the magnitude of the accelerations to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamped", py::overload_cast<double, double>(&JointAccelerations::clamp), "Return the accelerations clamped to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamp", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointAccelerations::clamp), "Clamp inplace the magnitude of the accelerations to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);
  c.def("clamped", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointAccelerations::clamp), "Return the accelerations clamped to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);

  c.def("__repr__", [](const JointAccelerations& accelerations) {
    std::stringstream buffer;
    buffer << accelerations;
    return buffer.str();
  });
}

void joint_torques(py::module_& m) {
  py::class_<JointTorques, JointState> c(m, "JointTorques");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and torque values provided", "robot_name"_a, "velocities"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and torque values provided", "robot_name"_a, "joint_names"_a, "velocities"_a);
  c.def(py::init<const JointTorques&>(), "Copy constructor", "torques"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointTorques::Zero), "Constructor for the zero JointTorques", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointTorques::Zero), "Constructor for the zero JointTorques", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointTorques::Random), "Constructor for the random JointTorques", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointTorques::Random), "Constructor for the random JointTorques", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "positions",
      "velocities",
      "accelerations",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointTorques&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointTorques& torques) -> JointTorques { return torques; }, "Deleted method from parent class.");
  }

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def(py::self -= py::self);
  c.def(py::self - py::self);
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(py::self *= Eigen::ArrayXd());
  c.def(py::self * Eigen::ArrayXd());
  c.def(py::self *= Eigen::MatrixXd());
  c.def(py::self * Eigen::MatrixXd());
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def(double() * py::self);
  c.def(Eigen::ArrayXd() * py::self);
  c.def(Eigen::MatrixXd() * py::self);

  c.def("copy", &JointTorques::copy, "Return a copy of the JointTorques");
  c.def("data", &JointTorques::data, "Returns the torques data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointTorques::set_data), "Set the torques data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointTorques::set_data), "Set the torques data from a list", "data"_a);

  c.def("clamp", py::overload_cast<double, double>(&JointTorques::clamp), "Clamp inplace the magnitude of the torque to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamped", py::overload_cast<double, double>(&JointTorques::clamp), "Return the torque clamped to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamp", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointTorques::clamp), "Clamp inplace the magnitude of the torque to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);
  c.def("clamped", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointTorques::clamp), "Return the torque clamped to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);

  c.def("__repr__", [](const JointTorques& torques) {
    std::stringstream buffer;
    buffer << torques;
    return buffer.str();
  });
}

void bind_joint_space(py::module_& m) {
  joint_state_variable(m);
  joint_state(m);
  joint_positions(m);
  joint_velocities(m);
  joint_torques(m);
}