#pragma once

#include <exception>
#include <iostream>

namespace StateRepresentation::Exceptions {
class IncompatibleReferenceFramesException : public std::logic_error {
public:
  explicit IncompatibleReferenceFramesException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::Exceptions
