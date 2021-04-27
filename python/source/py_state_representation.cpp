#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <eigen3/Eigen/Core>

#include <state_representation/State.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/JointState.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace state_representation;

PYBIND11_MODULE(py_state_representation, m) {
  m.doc() = "Python bindings for control_libraries state_representation";

  // CartesianState
  py::enum_<CartesianStateVariable>(m, "CartesianStateVariable")
      .value("POSITION", CartesianStateVariable::POSITION)
      .value("ORIENTATION", CartesianStateVariable::ORIENTATION)
      .value("POSE", CartesianStateVariable::POSE)
      .value("LINEAR_VELOCITY", CartesianStateVariable::LINEAR_VELOCITY)
      .value("ANGULAR_VELOCITY", CartesianStateVariable::ANGULAR_VELOCITY)
      .value("TWIST", CartesianStateVariable::TWIST)
      .value("LINEAR_ACCELERATION", CartesianStateVariable::LINEAR_ACCELERATION)
      .value("ANGULAR_ACCELERATION", CartesianStateVariable::ANGULAR_ACCELERATION)
      .value("ACCELERATIONS", CartesianStateVariable::ACCELERATIONS)
      .value("FORCE", CartesianStateVariable::FORCE)
      .value("TORQUE", CartesianStateVariable::TORQUE)
      .value("WRENCH", CartesianStateVariable::WRENCH)
      .value("ALL", CartesianStateVariable::ALL)
      .export_values();

  m.def("dist", py::overload_cast<const CartesianState&, const CartesianState&, const CartesianStateVariable&>(&dist), "Compute the distance between two CartesianStates", "s1"_a, "s2"_a, "state_variable_type"_a=CartesianStateVariable::ALL);

  py::class_<CartesianState> cartesian_state(m, "CartesianState");
  cartesian_state.def(py::init(), "Empty constructor");
  cartesian_state.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided", "name"_a, "reference"_a=std::string("world"));
  cartesian_state.def(py::init<const CartesianState&>(), "Copy constructor of a CartesianState", "state"_a);

  cartesian_state.def_static("Identity", &CartesianState::Identity, "Constructor for the identity CartesianState (identity pose and 0 for the rest)", "name"_a, "reference"_a=std::string("world"));
  cartesian_state.def_static("Random", &CartesianState::Random, "Constructor for a random state", "name"_a, "reference"_a=std::string("world"));

  cartesian_state.def("get_position", &CartesianState::get_position, "Getter of the position attribute");
  cartesian_state.def("get_orientation", &CartesianState::get_orientation_coefficients, "Getter of the orientation attribute as quaternion coefficients (w, x, y, z)");
  cartesian_state.def("get_pose", &CartesianState::get_pose, "Getter of a 7d pose vector from position and orientation coefficients");
  cartesian_state.def("get_transformation_matrix", &CartesianState::get_transformation_matrix, "Getter of a 4x4 transformation matrix of the pose");

  cartesian_state.def("get_linear_velocity", &CartesianState::get_linear_velocity, "Getter of the linear velocity attribute");
  cartesian_state.def("get_angular_velocity", &CartesianState::get_angular_velocity, "Getter of the angular velocity attribute");
  cartesian_state.def("get_twist", &CartesianState::get_twist, "Getter of the 6d twist from linear and angular velocity attributes");

  cartesian_state.def("get_linear_acceleration", &CartesianState::get_linear_acceleration, "Getter of the linear acceleration attribute");
  cartesian_state.def("get_angular_velocity", &CartesianState::get_angular_acceleration, "Getter of the linear acceleration attribute");
  cartesian_state.def("get_accelerations", &CartesianState::get_accelerations, "Getter of the 6d accelerations from linear and angular acceleration attributes");

  cartesian_state.def("get_force", &CartesianState::get_force, "Getter of the force attribute");
  cartesian_state.def("get_torque", &CartesianState::get_torque, "Getter of the torque attribute");
  cartesian_state.def("get_wrench", &CartesianState::get_wrench, "Getter of the 6d wrench from force and torque attributes");

  cartesian_state.def("set_position", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_position), "Setter of the position");
  cartesian_state.def("set_position", py::overload_cast<const std::vector<double>&>(&CartesianState::set_position), "Setter of the position from a list");
  cartesian_state.def("set_position", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_position), "Setter of the position from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  cartesian_state.def("set_orientation", py::overload_cast<const Eigen::Vector4d&>(&CartesianState::set_orientation), "Setter of the orientation from a 4d vector of quaternion coeffiecients (w, x, y, z)");
  cartesian_state.def("set_orientation", py::overload_cast<const std::vector<double>&>(&CartesianState::set_orientation), "Setter of the orientation from a 4d list of quaternion coeffiecients (w, x, y, z)");
  cartesian_state.def("set_pose", py::overload_cast<const Eigen::Matrix<double, 7, 1>&>(&CartesianState::set_pose), "Setter of the pose from a 7d vector of position and orientation coefficients (x, y, z, qw, qx, qy, qz");
  cartesian_state.def("set_pose", py::overload_cast<const std::vector<double>&>(&CartesianState::set_pose), "Setter of the pose from a 7d list of position and orientation coefficients (x, y, z, qw, qx, qy, qz");

  cartesian_state.def("set_linear_velocity", &CartesianState::set_linear_velocity, "Setter of the linear velocity attribute");
  cartesian_state.def("set_angular_velocity", &CartesianState::set_angular_velocity, "Setter of the angular velocity attribute");
  cartesian_state.def("set_twist", &CartesianState::set_twist, "Setter of the linear and angular velocities from a 6d twist vector");

  cartesian_state.def("set_linear_acceleration", &CartesianState::set_linear_acceleration, "Setter of the linear acceleration attribute");
  cartesian_state.def("set_angular_velocity", &CartesianState::set_angular_acceleration, "Setter of the linear acceleration attribute");
  cartesian_state.def("set_accelerations", &CartesianState::set_accelerations, "Setter of the linear and angular accelerations from a 6d acceleration vector");

  cartesian_state.def("set_force", &CartesianState::set_force, "Setter of the force attribute");
  cartesian_state.def("set_torque", &CartesianState::set_torque, "Setter of the torque attribute");
  cartesian_state.def("set_wrench", &CartesianState::set_wrench, "Setter of the force and torque from a 6d wrench vector");

  cartesian_state.def("set_zero", &CartesianState::set_zero, "Set the CartesianState to a zero value");
  cartesian_state.def("clamp_state_variable", &CartesianState::clamp_state_variable, "Clamp inplace the magnitude of the a specific state variable (velocity, acceleration or force)", "max_value"_a, "state_variable_type"_a, "noise_ratio"_a=double(0));
  cartesian_state.def("copy", &CartesianState::copy, "Return a copy of the CartesianState");
  cartesian_state.def("data", &CartesianState::data, "Returns the data as the concatenation of all the state variables in a single vector");
  cartesian_state.def("array", &CartesianState::array, "Returns the data vector as an array");

  cartesian_state.def(py::self *= py::self);
  cartesian_state.def(py::self * py::self);
  cartesian_state.def(py::self *= double());
  cartesian_state.def(py::self * double());
  cartesian_state.def(double() * py::self);
  cartesian_state.def(py::self += py::self);
  cartesian_state.def(py::self + py::self);
  cartesian_state.def(py::self -= py::self);
  cartesian_state.def(py::self - py::self);

  cartesian_state.def("inverse", &CartesianState::inverse, "Compute the inverse of the current CartesianState");
  cartesian_state.def("dist", &CartesianState::dist, "Compute the distance to another state as the sum of distances between each features", "state"_a, "state_variable_type"_a=CartesianStateVariable::ALL);
  cartesian_state.def("norms", &CartesianState::norms, "Compute the norms of the state variable specified by the input type (default is full state)", "state_variable_type"_a=CartesianStateVariable::ALL);
  cartesian_state.def("normalize", &CartesianState::normalize, "Normalize inplace the state at the state variable given in argument (default is full state)", "state_variable_type"_a=CartesianStateVariable::ALL);
  cartesian_state.def("normalized", &CartesianState::normalized, "Compute the normalized state at the state variable given in argument (default is full state)", "state_variable_type"_a=CartesianStateVariable::ALL);

  cartesian_state.def("to_list", &CartesianState::to_std_vector, "Return the state as a list");
  cartesian_state.def("from_list", &CartesianState::from_std_vector, "Set the state from a list");

  cartesian_state.def("__repr__", [](const CartesianState& state) {
      std::stringstream buffer;
      buffer << state;
      return buffer.str();
  });

  // JointState
  py::enum_<JointStateVariable>(m, "JointStateVariable")
      .value("POSITIONS", JointStateVariable::POSITIONS)
      .value("VELOCITIES", JointStateVariable::VELOCITIES)
      .value("ACCELERATIONS", JointStateVariable::ACCELERATIONS)
      .value("TORQUES", JointStateVariable::TORQUES)
      .value("ALL", JointStateVariable::ALL)
      .export_values();

  m.def("dist", py::overload_cast<const JointState&, const JointState&, const JointStateVariable&>(&dist), "Compute the distance between two JointStates", "s1"_a, "s2"_a, "state_variable_type"_a=JointStateVariable::ALL);

  py::class_<JointState> joint_state(m, "JointState");
  joint_state.def(py::init(), "Empty constructor");
  joint_state.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  joint_state.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  joint_state.def(py::init<const JointState&>(), "Copy constructor of a JointState", "state"_a);

  joint_state.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointState::Zero), "Constructor for the zero JointState", "robot_name"_a, "nb_joints"_a);
  joint_state.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointState::Zero), "Constructor for the zero JointState", "robot_name"_a, "joint_names"_a);
  joint_state.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointState::Random), "Constructor for the random JointState", "robot_name"_a, "nb_joints"_a);
  joint_state.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointState::Random), "Constructor for the random JointState", "robot_name"_a, "joint_names"_a);

  joint_state.def("get_size", &JointState::get_size, "Getter of the size from the attributes");
  joint_state.def("get_names", &JointState::get_names, "Getter of the names attribute");
  joint_state.def("get_positions", &JointState::get_positions, "Getter of the positions attribute");
  joint_state.def("get_velocities", &JointState::get_velocities, "Getter of the velocities attribute");
  joint_state.def("get_accelerations", &JointState::get_accelerations, "Getter of the accelerations attribute");
  joint_state.def("get_torques", &JointState::get_torques, "Getter of the torques attribute");

  joint_state.def("set_names", py::overload_cast<unsigned int>(&JointState::set_names), "Setter of the names attribute from the number of joints");
  joint_state.def("set_names", py::overload_cast<const std::vector<std::string>&>(&JointState::set_names), "Setter of the names attribute");
  joint_state.def("set_positions", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_positions), "Setter of the positions attribute from a vector");
  joint_state.def("set_positions", py::overload_cast<const std::vector<double>&>(&JointState::set_positions), "Setter of the positions attribute from a list");
  joint_state.def("set_velocities", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_velocities), "Setter of the velocities attribute from a vector");
  joint_state.def("set_velocities", py::overload_cast<const std::vector<double>&>(&JointState::set_velocities), "Setter of the velocities attribute from a list");
  joint_state.def("set_accelerations", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_accelerations), "Setter of the accelerations attribute from a vector");
  joint_state.def("set_accelerations", py::overload_cast<const std::vector<double>&>(&JointState::set_accelerations), "Setter of the accelerations attribute from a list");
  joint_state.def("set_torques", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_torques), "Setter of the torques attribute from a vector");
  joint_state.def("set_torques", py::overload_cast<const std::vector<double>&>(&JointState::set_torques), "Setter of the torques attribute from a list");

  joint_state.def("set_zero", &JointState::set_zero, "Set the JointState to a zero value");
  joint_state.def("clamp_state_variable", py::overload_cast<double, const JointStateVariable&, double>(&JointState::clamp_state_variable), "Clamp inplace the magnitude of the a specific state variable (velocities, accelerations or forces)", "value"_a, "state_variable_type"_a, "noise_ratio"_a=double(0));
  joint_state.def("clamp_state_variable", py::overload_cast<const Eigen::ArrayXd&, const JointStateVariable&, const Eigen::ArrayXd&>(&JointState::clamp_state_variable), "Clamp inplace the magnitude of the a specific state variable (velocities, accelerations or forces)", "max_absolute_value_array"_a, "state_variable_type"_a, "noise_ratio_array"_a);
  joint_state.def("copy", &JointState::copy, "Return a copy of the JointState");
  joint_state.def("data", &JointState::data, "Returns the data as the concatenation of all the state variables in a single vector");
  joint_state.def("array", &JointState::array, "Returns the data vector as an array");

  joint_state.def(py::self += py::self);
  joint_state.def(py::self + py::self);
  joint_state.def(py::self -= py::self);
  joint_state.def(py::self - py::self);
  joint_state.def(py::self *= double());
  joint_state.def(py::self * double());
  joint_state.def(py::self *= Eigen::ArrayXd());
  joint_state.def(py::self * Eigen::ArrayXd());
  joint_state.def(py::self *= Eigen::MatrixXd());
  joint_state.def(py::self * Eigen::MatrixXd());
  joint_state.def(py::self /= double());
  joint_state.def(py::self / double());
  joint_state.def(double() * py::self);
  joint_state.def(Eigen::ArrayXd() * py::self);
  joint_state.def(Eigen::MatrixXd() * py::self);

  joint_state.def("dist", &JointState::dist, "Compute the distance to another state as the sum of distances between each features", "state"_a, "state_variable_type"_a=JointStateVariable::ALL);

  joint_state.def("to_list", &JointState::to_std_vector, "Return the state as a list");
  joint_state.def("from_list", &JointState::from_std_vector, "Set the state from a list");

  joint_state.def("__repr__", [](const JointState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
}