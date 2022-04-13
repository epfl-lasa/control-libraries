#pragma once

#include <stdexcept>
#include <string>

namespace dynamical_systems::exceptions {
class InvalidDynamicalSystemException : public std::logic_error {
public:
  explicit InvalidDynamicalSystemException(const std::string& msg) : std::logic_error(msg) {};
};
}
