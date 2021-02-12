#include "state_representation/Parameters/Predicate.hpp"

namespace StateRepresentation {
Predicate::Predicate(const std::string& name) : Parameter<bool>(name, false) {}

Predicate::Predicate(const std::string& name, bool value) : Parameter<bool>(name, value) {}

std::ostream& operator<<(std::ostream& os, const Predicate& predicate) {
  os << " Predicate " << predicate.get_name() << ": " << predicate.get_value() << std::endl;
  return os;
}
}// namespace StateRepresentation