#include "state_representation/MathTools.hpp"

namespace state_representation::MathTools {
const Eigen::Quaterniond log(const Eigen::Quaterniond& q) {
  Eigen::Quaterniond log_q = Eigen::Quaterniond(0, 0, 0, 0);
  double q_norm = q.vec().norm();
  if (q_norm > 1e-4) { log_q.vec() = (q.vec() / q_norm) * acos(std::min<double>(std::max<double>(q.w(), -1), 1)); }
  return log_q;
}

const Eigen::Quaterniond exp(const Eigen::Quaterniond& q, double lambda) {
  Eigen::Quaterniond exp_q = Eigen::Quaterniond::Identity();
  double q_norm = q.vec().norm();
  if (q_norm > 1e-4) {
    exp_q.w() = cos(q_norm * lambda);
    exp_q.vec() = (q.vec() / q_norm) * sin(q_norm * lambda);
  }
  return exp_q;
}

const std::vector<double> linspace(double start, double end, unsigned int number_of_points) {
  // catch rarely, throw often
  if (number_of_points < 2) {
    throw new std::exception();
  }
  int partitions = number_of_points - 1;
  std::vector<double> pts;
  // length of each segment
  double length = (end - start) / partitions;
  // first, not to change
  pts.push_back(start);
  for (unsigned int i = 1; i < number_of_points - 1; i++) {
    pts.push_back(start + i * length);
  }
  // last, not to change
  pts.push_back(end);
  return pts;
}
}
