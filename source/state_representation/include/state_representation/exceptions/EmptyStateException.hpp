#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class EmptyStateException : public std::runtime_error {
public:
  explicit EmptyStateException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace state_representation::exceptions
