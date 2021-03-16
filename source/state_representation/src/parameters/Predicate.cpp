#include "state_representation/parameters/Predicate.hpp"

namespace state_representation {
Predicate::Predicate(const std::string& name) : Parameter<bool>(name, false) {}

Predicate::Predicate(const std::string& name, bool value) : Parameter<bool>(name, value) {}

std::ostream& operator<<(std::ostream& os, const Predicate& predicate) {
  os << " Predicate " << predicate.get_name() << ": " << predicate.get_value() << std::endl;
  return os;
}
}// namespace state_representation