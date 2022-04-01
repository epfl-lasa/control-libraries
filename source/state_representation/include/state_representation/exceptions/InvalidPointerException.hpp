#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class InvalidPointerException : public std::runtime_error {
public:
  explicit InvalidPointerException(const std::string& msg) : std::runtime_error(msg) {};
};
}
