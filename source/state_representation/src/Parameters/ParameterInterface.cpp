#include "state_representation/Parameters/ParameterInterface.hpp"

namespace StateRepresentation
{
	ParameterInterface::ParameterInterface(const StateType& type, const std::string& name):
	State(type, name)
	{}

	ParameterInterface::ParameterInterface(const ParameterInterface& parameter):
	State(parameter)
	{}
}