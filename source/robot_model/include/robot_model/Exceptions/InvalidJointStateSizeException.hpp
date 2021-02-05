#pragma once

#include <exception>
#include <iostream>

namespace RobotModel {
namespace Exceptions {
class InvalidJointStateSizeException : public std::runtime_error {
public:
  explicit InvalidJointStateSizeException(unsigned int state_nb_joints, int robot_nb_joints)
      : runtime_error("The robot has " + std::to_string(robot_nb_joints) + " joints, but the current joint state size "
                          + std::to_string(state_nb_joints) + ".") {};
};
}// namespace Exceptions
}// namespace RobotModel
