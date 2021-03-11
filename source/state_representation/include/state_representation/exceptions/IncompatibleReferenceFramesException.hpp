#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class IncompatibleReferenceFramesException : public std::logic_error {
public:
  explicit IncompatibleReferenceFramesException(const std::string& msg) : logic_error(msg) {};
};
}// namespace state_representation::exceptions
