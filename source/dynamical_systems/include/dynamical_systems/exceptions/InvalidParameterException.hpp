#pragma once

#include <exception>

namespace dynamical_systems::exceptions {
class InvalidParameterException : public std::runtime_error {
public:
  explicit InvalidParameterException(const std::string& msg) : runtime_error(msg) {};
};
}
