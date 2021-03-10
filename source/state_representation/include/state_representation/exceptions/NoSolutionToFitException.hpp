#pragma once

#include <exception>
#include <iostream>

namespace state_representation::Exceptions {
class NoSolutionToFitException : public std::runtime_error {
public:
  explicit NoSolutionToFitException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace StateRepresentation::exceptions
