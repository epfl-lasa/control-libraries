#pragma once

#include <exception>
#include <iostream>

namespace StateRepresentation::Exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::Exceptions
