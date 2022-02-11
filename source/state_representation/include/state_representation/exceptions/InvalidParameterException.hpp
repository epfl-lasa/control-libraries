#pragma once

#include <exception>

namespace state_representation::exceptions {
class InvalidParameterException : public std::runtime_error {
public:
  explicit InvalidParameterException(const std::string& msg) : runtime_error(msg) {};
};
}
