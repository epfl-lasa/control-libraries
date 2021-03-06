#pragma once

#include "filters/exceptions/NotImplementedException.hpp"
#include "state_representation/Parameters/ParameterInterface.hpp"
#include <list>
#include <memory>

namespace filters {
/**
 * @class Filter
 * @brief Abstract class to define a filter in any space
 * @tparam S the input type of the filter
 */
template<class S>
class Filter {
public:
  /**
   * @brief Empty constructor
   */
  explicit Filter();

  /**
   * @brief Apply the filter on an give input
   * To be redefined based on the actual filter implementation
   * @param input the input of the system.
   * @return S the filtered output
   */
  virtual S apply_filtering(const S& input);

  /**
   * @brief Return a list of all the parameters of the filter
   * @return the list of parameters
   */
  virtual std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> get_parameters() const;
};

template<class S>
Filter<S>::Filter() {}

template<class S>
std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Filter<S>::get_parameters() const {
  std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
  return param_list;
}

template<class S>
S Filter<S>::apply_filtering(const S& input) {
  throw exceptions::NotImplementedException("apply_filer(input) not implemented for the base filter class");
  return S();
}
}// namespace filters
