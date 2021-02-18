#pragma once

#include <exception>
#include <iostream>

namespace StateRepresentation::Exceptions {
class EmptyStateException : public std::runtime_error {
public:
  explicit EmptyStateException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace StateRepresentation::Exceptions
