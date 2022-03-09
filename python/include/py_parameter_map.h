#pragma once

#include <state_representation/parameters/ParameterMap.hpp>

class PyParameterMap : public ParameterMap, public std::enable_shared_from_this<PyParameterMap> {
public:
  using ParameterMap::ParameterMap;

protected:
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override {
    PYBIND11_OVERRIDE(void, ParameterMap, validate_and_set_parameter, parameter);
  }
};
