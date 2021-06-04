#pragma once

#include <iostream>
#include <exception>

namespace dynamical_systems::exceptions {
class EmptyBaseFrameException : public std::runtime_error {
public:
  explicit EmptyBaseFrameException(const std::string& msg) : runtime_error(msg) {};
};
}
