#pragma once

#include <stdexcept>
#include <string>

namespace controllers::exceptions {
class NoRobotModelException : public std::runtime_error {
public:
  explicit NoRobotModelException(const std::string& msg) : std::runtime_error(msg) {};
};
}
