#pragma once

#include <exception>
#include <iostream>

namespace RobotModel {
namespace Exceptions {
class FrameNotFoundException : public std::runtime_error {
public:
  explicit FrameNotFoundException(const std::string& frame_name) : runtime_error("Frame with name " + frame_name + " is not in the robot model"){};
};
}// namespace Exceptions
}// namespace RobotModel
