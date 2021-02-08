#pragma once

#include <exception>

namespace RobotModel::Exceptions {
class InvalidJointStateSizeException : public std::invalid_argument {
public:
  explicit InvalidJointStateSizeException(unsigned int state_nb_joints, unsigned int robot_nb_joints) :
      invalid_argument("The robot has " + std::to_string(robot_nb_joints) + " joints, but the current joint state size "
                           + std::to_string(state_nb_joints) + ".") {};
};
}// namespace RobotModel::Exceptions