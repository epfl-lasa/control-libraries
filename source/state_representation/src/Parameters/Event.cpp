#include "state_representation/Parameters/Event.hpp"

namespace state_representation {
Event::Event(const std::string& name) : Predicate(name), previous_predicate_value_(false) {}

std::ostream& operator<<(std::ostream& os, const Event& event) {
  os << " Event " << event.get_name() << ": " << event.get_value() << std::endl;
  return os;
}
}// namespace state_representation