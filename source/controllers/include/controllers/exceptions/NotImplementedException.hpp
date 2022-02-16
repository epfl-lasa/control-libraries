#pragma once

#include <stdexcept>
#include <string>

namespace controllers {
namespace exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : std::logic_error(msg) {};
};
}// namespace exceptions
}// namespace controllers
