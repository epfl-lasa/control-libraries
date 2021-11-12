#include "state_representation/space/SpatialState.hpp"

namespace state_representation {
SpatialState::SpatialState(const StateType& type) :
    State(type), reference_frame_("world") {}

SpatialState::SpatialState(const StateType& type,
                           const std::string& name,
                           const std::string& reference_frame,
                           const bool& empty) :
    State(type, name, empty), reference_frame_(reference_frame) {}

std::ostream& operator<<(std::ostream& os, const SpatialState& state) {
  if (state.is_empty()) {
    os << "Empty ";
  }
  os << " State: " << state.get_name() << " expressed in " << state.get_reference_frame() << " frame";
  return os;
}
}