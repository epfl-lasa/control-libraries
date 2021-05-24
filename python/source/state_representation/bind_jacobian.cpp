#include "state_representation_bindings.h"

#include <state_representation/State.hpp>
#include <state_representation/robot/Jacobian.hpp>
// TODO import necessary?
#include <state_representation/space/cartesian/CartesianPose.hpp>

void bind_jacobian(py::module_& m) {
  py::class_<Jacobian, State> c(m, "Jacobian");
  c.def(py::init<const std::string&, unsigned int, const std::string&, const std::string&>(),
        "Constructor with name, number of joints, frame name and reference frame provided",
        "robot_name"_a, "nb_joints"_a, "frame"_a, "reference_frame"_a = "world");
  c.def(py::init<const std::string&, const std::vector<std::string>&, const std::string&, const std::string&>(),
      "Constructor with name, joint names, frame name and reference frame provided",
      "robot_name"_a, "joint_names"_a, "frame"_a, "reference_frame"_a = "world");
  c.def(py::init<const std::string&, const std::string&, const Eigen::MatrixXd&, const std::string&>(),
        "Constructor with name, frame, Jacobian matrix and reference frame provided",
        "robot_name"_a, "frame"_a, "data"_a, "reference_frame"_a = "world");
  c.def(py::init<const std::string&, const std::vector<std::string>&, const std::string&, const Eigen::MatrixXd&, const std::string&>(),
        "Constructor with name, joint names, frame name, Jacobian matrix and reference frame provided",
        "robot_name"_a, "joint_names"_a, "frame"_a, "data"_a, "reference_frame"_a = "world");
  c.def(py::init<const Jacobian&>(), "Copy constructor of a Jacobian", "jacobian"_a);

  c.def_static("Random", py::overload_cast<const std::string&, unsigned int, const std::string&, const std::string&>(&Jacobian::Random),
      "Constructor for a random Jacobian",
      "robot_name"_a, "nb_joints"_a, "frame"_a, "reference_frame"_a = "world");
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&, const std::string&, const std::string&>(&Jacobian::Random),
      "Constructor for a random Jacobian",
      "robot_name"_a, "joint_names"_a, "frame"_a, "reference_frame"_a = "world");

  c.def("rows", &Jacobian::rows, "Getter of the number of rows attribute");
  c.def("cols", &Jacobian::cols, "Getter of the number of cols attribute");
  c.def("row", &Jacobian::row, "Accessor of the row data at given index");
  c.def("col", &Jacobian::col, "Accessor of the column data at given index");
  c.def("get_joint_names", &Jacobian::get_joint_names, "Getter of the joint_names attribute");
  c.def("get_frame", &Jacobian::get_frame, "Getter of the frame attribute");
  c.def("get_reference_frame", &Jacobian::get_frame, "Getter of the reference_frame attribute");
  c.def("data", &Jacobian::data, "Getter of the data attribute");

  c.def("set_rows", py::overload_cast<unsigned int>(&Jacobian::set_rows), "Setter of the number of rows"); // TODO why not param name
  c.def("set_cols", py::overload_cast<unsigned int>(&Jacobian::set_cols), "Setter of the number of columns");
  c.def("set_joint_names", py::overload_cast<unsigned int>(&Jacobian::set_joint_names), "Setter of the joint_names attribute from the number of joints");
  c.def("set_joint_names", py::overload_cast<const std::vector<std::string>&>(&Jacobian::set_joint_names), "Setter of the joint_names attribute from a vector of joint names");
  c.def("set_reference_frame", py::overload_cast<const CartesianPose&>(&Jacobian::set_reference_frame), "Setter of the reference_frame attribute from a CartesianPose");
  c.def("set_data", py::overload_cast<const Eigen::MatrixXd&>(&Jacobian::set_data), "Setter of the data attribute");

  // TODO what if its override
//  c.def("is_compatible", &Jacobian::is_compatible, "Set the JointState to a zero value");
//  c.def("initialize", &Jacobian::initialize), "Clamp inplace the magnitude of the a specific state variable (velocities, accelerations or forces)", "value"_a, "state_variable_type"_a, "noise_ratio"_a=double(0));
  c.def("transpose", &Jacobian::transpose, "Return the transpose of the Jacobian matrix");
  c.def("inverse", &Jacobian::inverse, "Return the inverse of the Jacobian matrix");
  c.def("pseudoinverse", &Jacobian::pseudoinverse, "Return the pseudoinverse of the Jacobian matrix");
  c.def("copy", &Jacobian::copy, "Return a copy of the Jacobian");
  c.def("copy", &Jacobian::copy, "Return a copy of the Jacobian");
  // TODO ???
//  c.def("solve", py::overload_cast<const Eigen::MatrixXd&>(&Jacobian::solve), "Solve the system X = inv(J)*M to obtain X which is more efficient than multiplying with the pseudo-inverse", "matrix"_a);
//  c.def("solve", py::overload_cast<const CartesianTwist&>(&Jacobian::solve), "Solve the system dX = J*dq to obtain dq which is more efficient than multiplying with the pseudo-inverse", "twist"_a);

  // TODO no empty constructor
  c.def(py::self * Eigen::MatrixXd());
//  c.def(py::self * Jacobian());
  c.def(py::self * JointVelocities());
  c.def(py::self * CartesianTwist());
  c.def(py::self * CartesianWrench());

  c.def("__repr__", [](const Jacobian& jacobian) {
    std::stringstream buffer;
    buffer << jacobian;
    return buffer.str();
  });
}

