#include "state_representation/geometry/Shape.hpp"

namespace state_representation {
Shape::Shape(const StateType& type) : State(type) {}

Shape::Shape(const StateType& type, const std::string& name, const std::string& reference_frame) :
    State(type, name),
    center_state_(CartesianPose(name, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), reference_frame)) {}

Shape::Shape(const Shape& shape) : State(shape), center_state_(shape.center_state_) {}

std::ostream& operator<<(std::ostream& os, const Shape& shape) {
  os << "Shape " << shape.get_name() << " with state:" << std::endl;
  os << shape.get_center_state();
  return os;
}
}