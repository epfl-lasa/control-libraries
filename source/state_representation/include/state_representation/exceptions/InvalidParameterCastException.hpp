#pragma once

#include "state_representation/exceptions/InvalidParameterException.hpp"

namespace state_representation::exceptions {
class InvalidParameterCastException : public InvalidParameterException {
public:
  explicit InvalidParameterCastException(const std::string& msg) : InvalidParameterException(msg) {};
};
}
