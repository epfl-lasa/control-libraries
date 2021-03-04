#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class IncompatibleStatesException : public std::logic_error {
public:
  explicit IncompatibleStatesException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::exceptions
