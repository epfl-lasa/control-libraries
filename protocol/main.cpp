#include <iostream>

#include <state_representation/space/cartesian/CartesianState.hpp>

#include "build/cartesian_state.pb.h"

int main() {
  std::cout << "Hello, World!" << std::endl;

  state_representation::CartesianState state("A", "B");

  state_representation::proto::CartesianState msg;
  msg.set_name(state.get_name());
  msg.set_reference_frame(state.get_reference_frame());

  std::cout << "Debug string:" << std::endl;
  msg.PrintDebugString();

  std::string output;
  msg.SerializeToString(&output);

  state_representation::proto::CartesianState msg_received;

  if (!msg_received.ParseFromString(output)) {
    std::cerr << "Could not parse string" << std::endl;
  } else {
    state_representation::CartesianState state_received(msg_received.name(), msg_received.reference_frame());
    std::cout << state_received << std::endl;
  }

  return 0;
}
