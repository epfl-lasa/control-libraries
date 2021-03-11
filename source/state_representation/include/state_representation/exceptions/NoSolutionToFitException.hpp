#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class NoSolutionToFitException : public std::runtime_error {
public:
  explicit NoSolutionToFitException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace state_representation::exceptions
