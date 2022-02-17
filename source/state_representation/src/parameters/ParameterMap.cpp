#include "state_representation/parameters/ParameterMap.hpp"

namespace state_representation {

ParameterMap::ParameterMap(const ParameterInterfaceList& parameters) {
  this->set_parameters(parameters);
}

ParameterMap::ParameterMap(const ParameterInterfaceMap& parameters) {
  this->set_parameters(parameters);
}

std::shared_ptr<ParameterInterface> ParameterMap::get_parameter(const std::string& name) const {
  if (this->parameters_.find(name) == this->parameters_.cend()) {
    throw exceptions::InvalidParameterException("Could not find a parameter named '" + name + "'.");
  }
  return this->parameters_.at(name);
}

ParameterInterfaceMap ParameterMap::get_parameters() const {
  return this->parameters_;
}

ParameterInterfaceList ParameterMap::get_parameter_list() const {
  ParameterInterfaceList param_list;
  for (const auto& param_it: this->parameters_) {
    param_list.template emplace_back(param_it.second);
  }
  return param_list;
}

void ParameterMap::set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  this->validate_and_set_parameter(parameter);
}

void ParameterMap::set_parameters(const ParameterInterfaceList& parameters) {
  for (const auto& param: parameters) {
    this->set_parameter(param);
  }
}

void ParameterMap::set_parameters(const ParameterInterfaceMap& parameters) {
  for (const auto& param_it: parameters) {
    this->set_parameter(param_it.second);
  }
}

void ParameterMap::assert_parameter_valid(const std::shared_ptr<ParameterInterface>& parameter) {
  if (this->parameters_.at(parameter->get_name())->get_type() != parameter->get_type()) {
    throw exceptions::InvalidParameterException(
        "Parameter '" + parameter->get_name() + "' exists, but has unexpected type.");
  }
}

void
ParameterMap::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  this->parameters_[parameter->get_name()] = parameter;
}

}