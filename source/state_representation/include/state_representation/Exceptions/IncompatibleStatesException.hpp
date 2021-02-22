#pragma once

#include <exception>
#include <iostream>

namespace StateRepresentation::Exceptions {
class IncompatibleStatesException : public std::logic_error {
public:
  explicit IncompatibleStatesException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::Exceptions
