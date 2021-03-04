#pragma once

#include "state_representation/parameters/Predicate.hpp"

namespace state_representation {
/**
 * @class Event
 * @brief An event is a predicate with memory. Its purpose is
 * to be true only once and change value only when the underlying
 * predicate has changed from true to false and back to true
 * since last reading
 */
class Event : public Predicate {
private:
  bool previous_predicate_value_;///< previous value of the predicate

public:
  /**
   * @brief Constructor with name of the predicate and default false value
   */
  explicit Event(const std::string& name);

  /**
   * @brief Read the value of the event, modifying its value as it has been
   * accessed once.
   */
  bool read_value();

  /**
   * @brief Setter of the value attribute
   * @param the new value attribute
   */
  void set_value(const bool& value) override;

  /**
   * @brief Getter of the previous value. Does not
   * affect the behavior of the event (as opposed to read_value)
   */
  bool get_previous_value() const;

  /**
    * @brief Overload the ostream operator for printing
    * @param os the ostream to happened the string representing the State to
    * @param predicate the Predicate to print
    * @return the appended ostream
     */
  friend std::ostream& operator<<(std::ostream& os, const Predicate& predicate);
};

inline bool Event::read_value() {
  bool value = this->get_value();
  this->Predicate::set_value(false);
  return value;
}

inline void Event::set_value(const bool& value) {
  bool current_value = this->get_value();
  bool result = value && (current_value || !this->previous_predicate_value_);
  this->previous_predicate_value_ = value;
  this->Predicate::set_value(result);
}

inline bool Event::get_previous_value() const {
  return this->previous_predicate_value_;
}
}// namespace state_representation