#pragma once

#include "state_representation/parameters/Parameter.hpp"

namespace state_representation {

/**
 * @class Predicate
 * @brief A predicate is a boolean parameter as in the logic formalism.
 */
class Predicate : public Parameter<bool> {
public:
  /**
   * @brief Constructor with name of the predicate and default false value.
   */
  explicit Predicate(const std::string& name);

  /**
   * @brief Constructor with a value.
   * @param name The name of the predicate
   * @param value Initial value of the predicate
   */
  explicit Predicate(const std::string& name, bool value);

  /**
   * @brief Overload the ostream operator for printing.
   * @param os The ostream to append the string representing the Predicate to
   * @param predicate The Predicate to print
   * @return the appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const Predicate& predicate);
};
}// namespace state_representation
