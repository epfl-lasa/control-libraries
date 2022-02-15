#pragma once

#include <exception>
#include <iostream>

namespace controllers {
namespace exceptions {
class InvalidControllerException : public std::logic_error {
public:
  explicit InvalidControllerException(const std::string& msg) : logic_error(msg){};
};
}// namespace exceptions
}// namespace controllers
