#pragma once

#include <exception>
#include <iostream>

namespace StateRepresentation::Exceptions {
class IncompatibleSizeException : public std::logic_error {
public:
  explicit IncompatibleSizeException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::Exceptions
