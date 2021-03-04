#pragma once

#include <cmath>

namespace state_representation::units {
class Distance;

inline namespace literals {
/**
 * @brief Literal operator to create a Distance in meter
 * @param n the distance value in meter
 * @param the Distance in the base unit (meter)
 */
constexpr Distance operator ""_m(long double n);

/**
 * @brief Literal operator to create a Distance in kilometer
 * @param n the distance value in kilometer
 * @param the Distance in the base unit (meter)
 */
constexpr Distance operator ""_km(long double n);

/**
 * @brief Literal operator to create a Distance in decimeter
 * @param n the distance value in decimeter
 * @param the Distance in the base unit (meter)
 */
constexpr Distance operator ""_dm(long double n);

/**
 * @brief Literal operator to create a Distance in centimeter
 * @param n the distance value in centimeter
 * @param the Distance in the base unit (meter)
 */
constexpr Distance operator ""_cm(long double n);

/**
 * @brief Literal operator to create a Distance in millimeter
 * @param n the distance value in millimeter
 * @param the Distance in the base unit (meter)
 */
constexpr Distance operator ""_mm(long double n);
}

class Distance {
private:
  long double value; ///< value of the distance in meter (base unit)

public:
  /**
   * @brief Constructor with a value in meter
   * @param n the value in meter
   */
  constexpr Distance(long double n = 0.0);

  /**
   * @brief Copy constructor from another distance
   * @param dist the distance to copy
   */
  constexpr Distance(const Distance& dist);

  /**
   * @brief Getter of the value attribute
   * @return the value in meter
   */
  constexpr long double get_value() const;

  /**
   * @brief Overload the - operator
   * @return the negative distance in meter
   */
  constexpr Distance& operator-();

  /**
   * @brief Overload the += operator
   * @param rhs Distance to add
   * @return the current Distance added the Distance given in argument
   */
  constexpr Distance& operator+=(const Distance& rhs);

  /**
   * @brief Overload the + operator
   * @param rhs Distance to add
   * @return the current Distance added the Distance given in argument
   */
  constexpr Distance operator+(const Distance& rhs) const;

  /**
   * @brief Overload the -= operator
   * @param rhs Distance to subtract
   * @return the current Distance minus the Distance given in argument
   */
  constexpr Distance& operator-=(const Distance& rhs);

  /**
   * @brief Overload the - operator
   * @param rhs Distance to subtract
   * @return the current Distance minus the Distance given in argument
   */
  constexpr Distance operator-(const Distance& rhs) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the Distance multiply by lambda
   */
  constexpr Distance& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda the scalar to multiply with
   * @return the Distance multiply by lambda
   */
  constexpr Distance operator*(double lambda) const;

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda the scalar to divide by
   * @return the Distance divided by lambda
   */
  constexpr Distance& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda the scalar to divide by
   * @return the Distance divided by lambda
   */
  constexpr Distance operator/(double lambda) const;

  /**
   * @brief Overload the == operator
   * @param rhs the other Distance to check equality with
   * @return bool true if the two distances are equal
   */
  constexpr bool operator==(const Distance& rhs) const;

  /**
   * @brief Overload the != operator
   * @param rhs the other Distance to check inequality with
   * @return bool true if the two distances are different
   */
  constexpr bool operator!=(const Distance& rhs) const;

  /**
   * @brief Overload the > operator
   * @param rhs the other Distance to check strict superiority with
   * @return bool true if the current distance is strictly superior than provided distance
   */
  constexpr bool operator>(const Distance& rhs) const;

  /**
   * @brief Overload the > operator
   * @param rhs the other Distance to check superiority with
   * @return bool true if the current distance is superior than provided distance
   */
  constexpr bool operator>=(const Distance& rhs) const;

  /**
   * @brief Overload the < operator
   * @param rhs the other Distance to check strict inferiority with
   * @return bool true if the current distance is strictly inferior than provided distance
   */
  constexpr bool operator<(const Distance& rhs) const;

  /**
   * @brief Overload the < operator
   * @param rhs the other Distance to check inferiority with
   * @return bool true if the current distance is inferior than provided distance
   */
  constexpr bool operator<=(const Distance& rhs) const;

  /**
   * @brief Overload the / operator between two distances
   * @param lhs the first distance
   * @param rhs the second distance
   * @return the ratio between first and second distance
   */
  friend constexpr double operator/(const Distance& lhs, const Distance& rhs);

  /**
   * @brief Overload the / operator with a scalar on the left side
   * @param lambda the scalar to multiply with
   * @return the Distance multiply by lambda
   */
  friend constexpr Distance operator*(double lambda, const Distance& rhs);

  /**
   * @brief Literal operator to create a Distance in meter
   * @param n the distance value in meter
   * @param the Distance in the base unit (meter)
   */
  friend constexpr Distance literals::operator ""_m(long double n);

  /**
   * @brief Literal operator to create a Distance in kilometer
   * @param n the distance value in kilometer
   * @param the Distance in the base unit (meter)
   */
  friend constexpr Distance literals::operator ""_km(long double n);

  /**
   * @brief Literal operator to create a Distance in decimeter
   * @param n the distance value in decimeter
   * @param the Distance in the base unit (meter)
   */
  friend constexpr Distance literals::operator ""_dm(long double n);

  /**
   * @brief Literal operator to create a Distance in centimeter
   * @param n the distance value in centimeter
   * @param the Distance in the base unit (meter)
   */
  friend constexpr Distance literals::operator ""_cm(long double n);

  /**
   * @brief Literal operator to create a Distance in millimeter
   * @param n the distance value in millimeter
   * @param the Distance in the base unit (meter)
   */
  friend constexpr Distance literals::operator ""_mm(long double n);
};

constexpr Distance::Distance(long double n) :
    value(n) {}

constexpr Distance::Distance(const Distance& dist) :
    value(dist.value) {}

constexpr long double Distance::get_value() const {
  return this->value;
}

constexpr Distance& Distance::operator-() {
  this->value = -this->value;
  return (*this);
}

constexpr Distance& Distance::operator+=(const Distance& rhs) {
  this->value = this->value + rhs.value;
  return (*this);
}

constexpr Distance Distance::operator+(const Distance& rhs) const {
  Distance result(*this);
  result += rhs;
  return result;
}

constexpr Distance& Distance::operator-=(const Distance& rhs) {
  this->value = this->value - rhs.value;
  return (*this);
}

constexpr Distance Distance::operator-(const Distance& rhs) const {
  Distance result(*this);
  result -= rhs;
  return result;
}

constexpr Distance& Distance::operator*=(double lambda) {
  this->value = this->value * lambda;
  return (*this);
}

constexpr Distance Distance::operator*(double lambda) const {
  Distance result(*this);
  result *= lambda;
  return result;
}

constexpr Distance& Distance::operator/=(double lambda) {
  this->value = this->value / lambda;
  return (*this);
}

constexpr Distance Distance::operator/(double lambda) const {
  Distance result(*this);
  result /= lambda;
  return result;
}

constexpr bool Distance::operator==(const Distance& rhs) const {
  return (abs(this->value - rhs.value) < 1e-4);
}

constexpr bool Distance::operator!=(const Distance& rhs) const {
  return !((*this) == rhs);
}

constexpr bool Distance::operator>(const Distance& rhs) const {
  return ((this->value - rhs.value) > 1e-4);
}

constexpr bool Distance::operator>=(const Distance& rhs) const {
  return (((*this) > rhs) or ((*this) == rhs));
}

constexpr bool Distance::operator<(const Distance& rhs) const {
  return ((rhs.value - this->value) > 1e-4);
}

constexpr bool Distance::operator<=(const Distance& rhs) const {
  return (((*this) < rhs) or ((*this) == rhs));
}

constexpr double operator/(const Distance& lhs, const Distance& rhs) {
  return lhs.get_value() / rhs.get_value();
}

constexpr Distance operator*(double lambda, const Distance& rhs) {
  return Distance(lambda * rhs.value);
}

inline namespace literals {
constexpr Distance operator ""_m(long double n) {
  return Distance(n);
}

constexpr Distance operator ""_km(long double n) {
  return Distance(1e3 * n);
}

constexpr Distance operator ""_dm(long double n) {
  return Distance(1e-1 * n);
}

constexpr Distance operator ""_cm(long double n) {
  return Distance(1e-2 * n);
}

constexpr Distance operator ""_mm(long double n) {
  return Distance(1e-3 * n);
}
}
}
