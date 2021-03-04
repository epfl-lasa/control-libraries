#pragma once

#include <exception>
#include <iostream>

namespace state_representation::Exceptions {
class IncompatibleReferenceFramesException : public std::logic_error {
public:
  explicit IncompatibleReferenceFramesException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::exceptions
