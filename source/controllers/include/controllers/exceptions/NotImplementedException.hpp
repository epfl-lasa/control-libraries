#pragma once

#include <stdexcept>
#include <string>

namespace controllers::exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : std::logic_error(msg) {};
};
}
