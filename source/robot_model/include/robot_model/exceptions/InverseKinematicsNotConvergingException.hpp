#pragma once

#include <exception>

namespace robot_model::exceptions {
class InverseKinematicsNotConvergingException : public std::exception {
public:
  InverseKinematicsNotConvergingException(unsigned int iterations, double error) :
    msg_("The inverse kinematics algorithm did not converge.\nThe residual error after " + std::to_string(iterations) +
         " iterations is " + std::to_string(error) + ".") {}

   virtual char const *what() const noexcept {
     return msg_.c_str();
   }

private:
  std::string msg_;
};
}// namespace robot_model::exceptions
