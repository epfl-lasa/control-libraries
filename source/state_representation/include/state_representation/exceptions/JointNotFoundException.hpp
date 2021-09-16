#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {

/**
 * @class JointNotFoundException
 * @brief Exception that is thrown when a joint name or index is out of range
 */
class JointNotFoundException : public std::logic_error {
public:
  explicit JointNotFoundException(const std::string& msg) : logic_error(msg) {};
};
}// namespace state_representation::exceptions
