#pragma once

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace state_representation::MathTools {
/**
 * @brief Calculate the log of a quaternion as a non-unit quaternion
 * @param  q the quaternion to apply the log on
 * @return the log of the quaternion
 */
const Eigen::Quaterniond log(const Eigen::Quaterniond& q);

/**
 * @brief Calculate the exp of a quaternion as a non-unit quaternion
 * @param  q the quaternion to apply the exp on
 * @return the exp of the quaternion
 */
const Eigen::Quaterniond exp(const Eigen::Quaterniond& q, double lambda = 1);

/**
 * @brief Create a vector values from start to end
 * @param start the starting value
 * @param end the end value
 * @param number_of_points the total number of points
 */
const std::vector<double> linspace(double start, double end, unsigned int number_of_points);
}
