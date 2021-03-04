#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::exceptions
