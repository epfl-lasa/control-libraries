#include "robot_model_bindings.h"

#include <robot_model/Model.hpp>

void model(py::module_& m) {
  m.def("create_urdf_from_string", &Model::create_urdf_from_string, "Creates a URDF file with desired path and name from a string (possibly the robot description string from the ROS parameter server).", "urdf_string"_a, "desired_path"_a);

  py::class_<Model> c(m, "Model");

  c.def(py::init<const std::string&, const std::string&>(), "Constructor with robot name and path to URDF file.", "robot_name"_a, "urdf_path"_a);
  c.def(py::init<const Model&>(), "Copy constructor from another Model", "model"_a);

  c.def("get_robot_name", &Model::get_robot_name, "Getter of the robot name.");
  c.def("set_robot_name", &Model::set_robot_name, "Setter of the robot name.", "robot_name"_a);
  c.def("get_urdf_path", &Model::get_urdf_path, "Getter of the URDF path.");
  c.def("get_number_of_joints", &Model::get_number_of_joints, "Getter of the number of joints.");
  c.def("get_joint_frames", &Model::get_joint_frames, "Getter of the joint frames of the model.");
  c.def("get_frames", &Model::get_frames, "Getter of the frames of the model.");
  c.def("get_base_frame", &Model::get_base_frame, "Getter of the base frame of the model.");
  c.def("get_gravity_vector", &Model::get_gravity_vector, "Getter of the gravity vector.");
  c.def("set_gravity_vector", &Model::set_gravity_vector, "Setter of the gravity vector.", "gravity"_a);
//  c.def("get_pinocchio_model", &Model::get_pinocchio_model, "Getter of the pinocchio model.");

  c.def(
      "compute_jacobian", py::overload_cast<const state_representation::JointPositions&, const std::string&>(&Model::compute_jacobian),
      "Compute the Jacobian from a given joint state at the frame given in parameter.", "joint_positions"_a, "frame_name"_a = std::string(""));
  c.def(
      "compute_jacobian_time_derivative", py::overload_cast<const state_representation::JointPositions&, const state_representation::JointVelocities&, const std::string&>(&Model::compute_jacobian_time_derivative),
      "Compute the time derivative of the Jacobian from given joint positions and velocities at the frame in parameter.", "joint_positions"_a, "joint_velocities"_a, "frame_name"_a = std::string(""));
  c.def("compute_inertia_matrix", py::overload_cast<const state_representation::JointPositions&>(&Model::compute_inertia_matrix), "Compute the Inertia matrix from given joint positions.", "joint_positions"_a);
  c.def(
      "compute_inertia_torques", py::overload_cast<const state_representation::JointState&>(&Model::compute_inertia_torques),
      "Compute the Inertia torques, i.e the inertia matrix multiplied by the joint accelerations. Joint positions are needed as well for computations of the inertia matrix.", "joint_state"_a);
  c.def("compute_coriolis_matrix", py::overload_cast<const state_representation::JointState&>(&Model::compute_coriolis_matrix), "Compute the Coriolis matrix from a given joint state.", "joint_state"_a);
  c.def(
      "compute_coriolis_torques", py::overload_cast<const state_representation::JointState&>(&Model::compute_coriolis_torques),
      "Compute the Coriolis torques, i.e. the Coriolis matrix multiplied by the joint velocities and express the result as a JointTorques.", "joint_state"_a);
  c.def("compute_gravity_torques", py::overload_cast<const state_representation::JointPositions&>(&Model::compute_gravity_torques), "Compute the gravity torques.", "joint_positions"_a);

  c.def("print_qp_problem", &Model::print_qp_problem, "Helper function to print the qp problem (for debugging).");
//  c.def(
//      "clamp_in_range", py::overload_cast<const state_representation::JointState&>(&Model::clamp_in_range),
//      "Clamp the joint state variables (positions, velocities & torques) according to the limits provided by the model", "joint_state"_a);
}

void bind_model(py::module_& m) {
  model(m);
}