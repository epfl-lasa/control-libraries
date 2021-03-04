#pragma once

#include <cmath>

namespace state_representation::Units {
class Angle;

inline namespace literals {
/**
 * @brief Literal operator to create an Angle in radian
 * @param n the angle value in radian
 * @param the Angle in the base unit (radian)
 */
constexpr Angle operator ""_rad(long double n);

/**
 * @brief Literal operator to create an Angle in degrees
 * @param n the angle value in degree
 * @param the Angle in the base unit (radian)
 */
constexpr Angle operator ""_deg(long double n);
}

class Angle {
private:
  long double value; ///< value of the angle in radian (base unit)

public:
  /**
   * @brief Constructor with a value in radian in [-pi,pi]
   * @param n the value in radian
   */
  constexpr Angle(long double n = 0.0);

  /**
   * @brief Copy constructor from another Angle
   * @param ang the Angle to copy
   */
  constexpr Angle(const Angle& ang);

  /**
   * @brief Getter of the value attribute
   * @return the value in radian
   */
  constexpr long double get_value() const;

  /**
   * @brief Overload the = operator
   * @param n the angle value to assign in radian
   * @return the angle in radian
   */
  constexpr Angle& operator=(long double n);

  /**
   * @brief Overload the - operator
   * @return the negative angle in radian
   */
  constexpr Angle& operator-();

  /**
   * @brief Overload the += operator
   * @param rhs Angle to add
   * @return the current Angle added the Angle given in argument
   */
  constexpr Angle& operator+=(const Angle& rhs);

  /**
   * @brief Overload the + operator
   * @param rhs Angle to add
   * @return the current Angle added the Angle given in argument
   */
  constexpr Angle operator+(const Angle& rhs) const;

  /**
   * @brief Overload the -= operator
   * @param rhs Angle to subtract
   * @return the current Angle minus the Angle given in argument
   */
  constexpr Angle& operator-=(const Angle& rhs);

  /**
   * @brief Overload the - operator
   * @param rhs Angle to subtract
   * @return the current Angle minus the Angle given in argument
   */
  constexpr Angle operator-(const Angle& rhs) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the Angle multiply by lambda
   */
  constexpr Angle& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the Angle multiply by lambda
   */
  constexpr Angle operator*(double lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide by
   * @return the Angle divided by lambda
   */
  constexpr Angle& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide by
   * @return the Angle divided by lambda
   */
  constexpr Angle operator/(double lambda) const;

  /**
   * @brief Overload the == operator
   * @param rhs the other Angle to check equality with
   * @return bool true if the two Angles are equal
   */
  constexpr bool operator==(const Angle& rhs) const;

  /**
   * @brief Overload the != operator
   * @param rhs the other Angle to check inequality with
   * @return bool true if the two Angles are different
   */
  constexpr bool operator!=(const Angle& rhs) const;

  /**
   * @brief Overload the > operator
   * @param rhs the other Angle to check strict superiority with
   * @return bool true if the current Angle is strictly superior than provided Angle
   */
  constexpr bool operator>(const Angle& rhs) const;

  /**
   * @brief Overload the > operator
   * @param rhs the other Angle to check superiority with
   * @return bool true if the current Angle is superior than provided Angle
   */
  constexpr bool operator>=(const Angle& rhs) const;

  /**
   * @brief Overload the < operator
   * @param rhs the other Angle to check strict inferiority with
   * @return bool true if the current Angle is strictly inferior than provided Angle
   */
  constexpr bool operator<(const Angle& rhs) const;

  /**
   * @brief Overload the < operator
   * @param rhs the other Angle to check inferiority with
   * @return bool true if the current Angle is inferior than provided Angle
   */
  constexpr bool operator<=(const Angle& rhs) const;

  /**
   * @brief Overload the / operator between two Angles
   * @param lhs the first Angle
   * @param rhs the second Angle
   * @return the ratio between first and second Angle
   */
  friend constexpr double operator/(const Angle& lhs, const Angle& rhs);

  /**
   * @brief Overload the / operator with a scalar on the left side
   * @param lambda the scalar to multiply with
   * @return the Angle multiply by lambda
   */
  friend constexpr Angle operator*(double lambda, const Angle& rhs);

  /**
   * @brief Literal operator to create an Angle in radian
   * @param n the angle value in radian
   * @param the Angle in the base unit (radian)
   */
  friend constexpr Angle literals::operator ""_rad(long double n);

  /**
   * @brief Literal operator to create an Angle in degrees
   * @param n the angle value in degree
   * @param the Angle in the base unit (radian)
   */
  friend constexpr Angle literals::operator ""_deg(long double n);
};

constexpr Angle::Angle(long double n) :
    value(atan2(sin(n), cos(n))) {}

constexpr Angle::Angle(const Angle& ang) :
    value(ang.value) {}

constexpr long double Angle::get_value() const {
  return this->value;
}

constexpr Angle& Angle::operator=(long double n) {
  this->value = atan2(sin(n), cos(n));
  return (*this);
}

constexpr Angle& Angle::operator-() {
  this->value = -this->value;
  return (*this);
}

constexpr Angle& Angle::operator+=(const Angle& rhs) {
  double n = this->value + rhs.value;
  this->value = atan2(sin(n), cos(n));
  return (*this);
}

constexpr Angle Angle::operator+(const Angle& rhs) const {
  Angle result(*this);
  result += rhs;
  return result;
}

constexpr Angle& Angle::operator-=(const Angle& rhs) {
  double n = this->value - rhs.value;
  this->value = atan2(sin(n), cos(n));
  return (*this);
}

constexpr Angle Angle::operator-(const Angle& rhs) const {
  Angle result(*this);
  result -= rhs;
  return result;
}

constexpr Angle& Angle::operator*=(double lambda) {
  double n = this->value * lambda;
  this->value = atan2(sin(n), cos(n));
  return (*this);
}

constexpr Angle Angle::operator*(double lambda) const {
  Angle result(*this);
  result *= lambda;
  return result;
}

constexpr Angle& Angle::operator/=(double lambda) {
  double n = this->value / lambda;
  this->value = atan2(sin(n), cos(n));
  return (*this);
}

constexpr Angle Angle::operator/(double lambda) const {
  Angle result(*this);
  result /= lambda;
  return result;
}

constexpr bool Angle::operator==(const Angle& rhs) const {
  return (abs(this->value - rhs.value) < 1e-4);
}

constexpr bool Angle::operator!=(const Angle& rhs) const {
  return !((*this) == rhs);
}

constexpr bool Angle::operator>(const Angle& rhs) const {
  return ((this->value - rhs.value) > 1e-4);
}

constexpr bool Angle::operator>=(const Angle& rhs) const {
  return (((*this) > rhs) or ((*this) == rhs));
}

constexpr bool Angle::operator<(const Angle& rhs) const {
  return ((rhs.value - this->value) > 1e-4);
}

constexpr bool Angle::operator<=(const Angle& rhs) const {
  return (((*this) < rhs) or ((*this) == rhs));
}

constexpr double operator/(const Angle& lhs, const Angle& rhs) {
  return lhs.get_value() / rhs.get_value();
}

constexpr Angle operator*(double lambda, const Angle& rhs) {
  return Angle(lambda * rhs.value);
}

inline namespace literals {
constexpr Angle operator ""_rad(long double n) {
  return Angle(n);
}

constexpr Angle operator ""_deg(long double n) {
  return Angle(M_PI * n / 180);
}
}
}