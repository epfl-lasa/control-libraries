/**
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#pragma once

#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <random>
#include "state_representation/geometry/Shape.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"

namespace state_representation {
/**
 * @class Ellipsoid
 */
class Ellipsoid : public Shape {
private:
  std::vector<double> axis_lengths_; ///< axis lengths in x,y directions
  double rotation_angle_; ///< angle of rotation around z axis of the reference frame

public:
  /**
   * @brief Constructor with name but empty state
   * @param name name of the ellipsoid
   * @param safety_margin the safety_margin (default=0 in all axes)
   */
  explicit Ellipsoid(const std::string& name, const std::string& reference_frame = "world");

  /**
   * @brief Copy constructor from another ellipsoid
   * @param ellipsoid the ellipsoid to copy
   */
  Ellipsoid(const Ellipsoid& ellipsoid);

  /**
   * @brief Copy assignment operator that have to be defined to the custom assignment operator
   * @param state the state with value to assign
   * @return reference to the current state with new values
   */
  Ellipsoid& operator=(const Ellipsoid& state);

  /**
   * @brief Getter of the axis lengths
   * @return the axis lengths
   */
  const std::vector<double>& get_axis_lengths() const;

  /**
   * @brief Getter of the axis length in one direction
   * @param index the index of the length (0 for x, 1 for y and 2 for z)
   * @return the length in the desired direction
   */
  double get_axis_length(unsigned int index) const;

  /**
   * @brief Setter of the axis lengths
   * @param axis_lengths the new values
   */
  void set_axis_lengths(const std::vector<double>& axis_lengths);

  /**
   * @brief Setter of the axis length at given index
   * @param index the desired index
   * @param axis_length the new length
   */
  void set_axis_lengths(unsigned int index, double axis_length);

  /**
   * @brief Getter of the rotation angle
   * @return the rotation angle
   */
  double get_rotation_angle() const;

  /**
   * @brief Seetter of the rotation angle
   * @param rotation_angle the rotation angle
   */
  void set_rotation_angle(double rotation_angle);

  const CartesianPose get_rotation() const;

  /**
   * @brief Function to sample an obstacle from its parameterization
   * @param nb_samples the number of sample points to generate
   * @return the list of sample points
   */
  const std::list<CartesianPose> sample_from_parameterization(unsigned int nb_samples) const;

  /**
   * @brief Compute an ellipsoid from its algebraic equation ax2 + bxy + cy2 + cx + ey + f
   * @return the Ellipsoid in its geometric representation
   */
  static const Ellipsoid from_algebraic_equation(const std::string& name,
                                                 const std::vector<double>& coefficients,
                                                 const std::string& reference_frame = "world");

  /**
   * @brief Fit an ellipsoid on a set of points
   * This method uses direct least square fitting from
   * Fitzgibbon, A., et al. (1999). "Direct least square fitting of ellipses."
    * IEEE Transactions on pattern analysis and machine intelligence 21(5)
   */
  static const Ellipsoid fit(const std::string& name,
                             const std::list<CartesianPose>& points,
                             const std::string& reference_frame = "world",
                             double noise_level = 0.01);

  /**
   * @brief Convert the ellipse to an std vector representation of its parameter
   * @return an std vector with [center_position, rotation_angle, axis_lengths]
   */
  const std::vector<double> to_std_vector() const;

  /**
   * @brief Create an ellipsoid from an std vector representaiton of its parameter
   * @param an std vector with [center_position, rotation_angle, axis_lengths]
   */
  void from_std_vector(const std::vector<double>& parameters);

  /**
    * @brief Overload the ostream operator for printing
    * @param os the ostream to happend the string representing the Ellipsoid to
    * @param ellipsoid the Ellipsoid to print
    * @return the appended ostream
     */
  friend std::ostream& operator<<(std::ostream& os, const Ellipsoid& ellipsoid);
};

inline Ellipsoid& Ellipsoid::operator=(const Ellipsoid& state) {
  Shape::operator=(state);
  this->set_axis_lengths(state.get_axis_lengths());
  this->set_rotation_angle(state.get_rotation_angle());
  return (*this);
}

inline const std::vector<double>& Ellipsoid::get_axis_lengths() const {
  return this->axis_lengths_;
}

inline double Ellipsoid::get_axis_length(unsigned int index) const {
  return this->axis_lengths_[index];
}

inline void Ellipsoid::set_axis_lengths(const std::vector<double>& axis_lengths) {
  this->axis_lengths_ = axis_lengths;
  this->set_filled();
}

inline void Ellipsoid::set_axis_lengths(unsigned int index, double axis_length) {
  this->axis_lengths_[index] = axis_length;
  this->set_filled();
}

inline double Ellipsoid::get_rotation_angle() const {
  return this->rotation_angle_;
}

inline void Ellipsoid::set_rotation_angle(double rotation_angle) {
  this->rotation_angle_ = rotation_angle;
  this->set_filled();
}

inline const std::vector<double> Ellipsoid::to_std_vector() const {
  std::vector<double> representation(6);
  // position
  representation[0] = this->get_center_position()(0);
  representation[1] = this->get_center_position()(1);
  representation[2] = this->get_center_position()(2);
  // rotation angle
  representation[3] = this->get_rotation_angle();
  // axis lengths
  representation[4] = this->get_axis_length(0);
  representation[5] = this->get_axis_length(1);
  return representation;
}

inline void Ellipsoid::from_std_vector(const std::vector<double>& parameters) {
  this->set_center_position(Eigen::Vector3d(parameters[0], parameters[1], parameters[2]));
  this->set_rotation_angle(parameters[3]);
  this->set_axis_lengths({parameters[4], parameters[5]});
}

inline const CartesianPose Ellipsoid::get_rotation() const {
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(this->rotation_angle_, Eigen::Vector3d::UnitZ()));
  return CartesianPose(this->get_center_pose().get_name() + "_rotated",
                       Eigen::Vector3d::Zero(),
                       rotation,
                       this->get_center_pose().get_name());
}
}