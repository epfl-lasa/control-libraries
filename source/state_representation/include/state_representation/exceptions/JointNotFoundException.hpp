#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class JointNotFoundException : public std::logic_error {
public:
  explicit JointNotFoundException(const std::string& msg) : logic_error(msg) {};
};
}// namespace state_representation::exceptions
