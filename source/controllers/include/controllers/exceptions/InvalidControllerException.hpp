#pragma once

#include <stdexcept>
#include <string>

namespace controllers::exceptions {
class InvalidControllerException : public std::logic_error {
public:
  explicit InvalidControllerException(const std::string& msg) : std::logic_error(msg) {};
};
}
