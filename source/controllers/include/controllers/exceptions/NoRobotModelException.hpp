#pragma once

#include <stdexcept>
#include <string>

namespace controllers {
namespace exceptions {
class NoRobotModelException : public std::runtime_error {
public:
  explicit NoRobotModelException(const std::string& msg) : std::runtime_error(msg) {};
};
}// namespace exceptions
}// namespace controllers
