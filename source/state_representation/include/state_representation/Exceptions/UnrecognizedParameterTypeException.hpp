#pragma once

#include <exception>
#include <iostream>

namespace StateRepresentation::Exceptions {
class UnrecognizedParameterTypeException : public std::logic_error {
public:
  explicit UnrecognizedParameterTypeException(const std::string& msg) : logic_error(msg) {};
};
}// namespace StateRepresentation::Exceptions
