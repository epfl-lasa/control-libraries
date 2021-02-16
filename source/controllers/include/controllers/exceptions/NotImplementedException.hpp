#pragma once

#include <exception>
#include <iostream>

namespace controllers {
namespace exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : logic_error(msg){};
};
}// namespace exceptions
}// namespace controllers
