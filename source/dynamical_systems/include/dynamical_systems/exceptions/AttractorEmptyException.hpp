#pragma once

#include <iostream>
#include <exception>

namespace dynamical_systems::exceptions {
class AttractorEmptyException : public std::runtime_error {
public:
  explicit AttractorEmptyException(const std::string& msg) : runtime_error(msg) {};
};
}
