#pragma once

#include <exception>
#include <iostream>

namespace controllers {
namespace exceptions {
class NoRobotModelException : public std::runtime_error {
public:
  explicit NoRobotModelException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace exceptions
}// namespace controllers
