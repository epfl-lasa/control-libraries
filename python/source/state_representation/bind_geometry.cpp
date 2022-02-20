#include "state_representation_bindings.h"

#include <state_representation/State.hpp>
#include <state_representation/geometry/Shape.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>


void shape(py::module_& m) {
  py::class_<Shape, std::shared_ptr<Shape>, State> c(m, "Shape");

  c.def(py::init<const StateType&, const std::string&, const std::string&>(), "Constructor with name but empty state.", "type"_a, "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const Shape&>(), "Copy constructor from another Shape.", "shape"_a);

  c.def("get_center_state", &Shape::get_center_state, "Getter of the state.");
  c.def("get_center_pose", &Shape::get_center_pose, "Getter of the pose from the state.");
  c.def("get_center_position", &Shape::get_center_position, "Getter of the position from the state.");
  c.def("get_center_orientation", &Shape::get_center_orientation, "Getter of the orientation from the state.");
  c.def("get_center_twist", &Shape::get_center_twist, "Getter of the twist from the state.");

  c.def("set_center_state", &Shape::set_center_state, "Setter of the state.");
  c.def("set_center_pose", &Shape::set_center_pose, "Setter of the pose from the state.");
  c.def("set_center_position", &Shape::set_center_position, "Setter of the position from the state.");
  c.def("set_center_orientation", &Shape::set_center_orientation, "Setter of the orientation from the state.");

  c.def("__copy__", [](const Shape &shape) {
    return Shape(shape);
  });
  c.def("__deepcopy__", [](const Shape &shape, py::dict) {
    return Shape(shape);
  }, "memo"_a);
  c.def("__repr__", [](const Shape& shape) {
    std::stringstream buffer;
    buffer << shape;
    return buffer.str();
  });
}

void ellipsoid(py::module_& m) {
  py::class_<Ellipsoid, std::shared_ptr<Ellipsoid>, Shape> c(m, "Ellipsoid");

  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name but empty state.", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const Ellipsoid&>(), "Copy constructor from another Ellipsoid.", "ellipsoid"_a);

  c.def("get_axis_lengths", &Ellipsoid::get_axis_lengths, "Getter of the axis lengths.");
  c.def("get_axis_length", &Ellipsoid::get_axis_length, "Getter of the axis length in one direction.", "index"_a);
  c.def("set_axis_lengths", py::overload_cast<const std::vector<double>&>(&Ellipsoid::set_axis_lengths), "Setter of the axis lengths.", "axis_lengths"_a);
  c.def("set_axis_lengths", py::overload_cast<unsigned int, double>(&Ellipsoid::set_axis_lengths), "Setter of the axis length at given index.", "index"_a, "axis_length"_a);
  c.def("get_rotation_angle", &Ellipsoid::get_rotation_angle, "Getter of the rotation angle.");
  c.def("set_rotation_angle", &Ellipsoid::set_rotation_angle, "Setter of the rotation angle.", "rotation_angle"_a);

  c.def("get_rotation", &Ellipsoid::get_rotation, "Getter of the rotation.");
  c.def("sample_from_parameterization", &Ellipsoid::sample_from_parameterization, "Function to sample an obstacle from its parameterization.", "nb_samples"_a);
  c.def("from_algebraic_equation", &Ellipsoid::from_algebraic_equation, "Compute an ellipsoid from its algebraic equation ax2 + bxy + cy2 + cx + ey + f.");
  c.def("fit", &Ellipsoid::fit, "Fit an ellipsoid on a set of points.");

  c.def("to_std_vector", &Ellipsoid::to_std_vector, "Convert the ellipse to an std vector representation of its parameter.");

  c.def("__copy__", [](const Shape &shape) {
    return Shape(shape);
  });
  c.def("__deepcopy__", [](const Shape &shape, py::dict) {
    return Shape(shape);
  }, "memo"_a);
  c.def("__repr__", [](const Shape& shape) {
    std::stringstream buffer;
    buffer << shape;
    return buffer.str();
  });
}

void bind_geometry(py::module_& m) {
  shape(m);
  ellipsoid(m);
}
