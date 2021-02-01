#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Geometry/Ellipsoid.hpp"

namespace StateRepresentation
{
	template <>
	Parameter<double>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_DOUBLE, name)
	{
		this->set_filled();
	}

	template <>
	Parameter<double>::Parameter(const std::string& name, const double& value):
	ParameterInterface(StateType::PARAMETER_DOUBLE, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::vector<double>>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_DOUBLE_ARRAY, name)
	{}

	template <>
	Parameter<std::vector<double>>::Parameter(const std::string& name, const std::vector<double>& value):
	ParameterInterface(StateType::PARAMETER_DOUBLE_ARRAY, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<bool>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_BOOL, name)
	{}

	template <>
	Parameter<bool>::Parameter(const std::string& name, const bool& value):
	ParameterInterface(StateType::PARAMETER_BOOL, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::vector<bool>>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_BOOL_ARRAY, name)
	{}

	template <>
	Parameter<std::vector<bool>>::Parameter(const std::string& name, const std::vector<bool>& value):
	ParameterInterface(StateType::PARAMETER_BOOL_ARRAY, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::string>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_STRING, name)
	{}

	template <>
	Parameter<std::string>::Parameter(const std::string& name, const std::string& value):
	ParameterInterface(StateType::PARAMETER_STRING, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<std::vector<std::string>>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_STRING_ARRAY, name)
	{}

	template <>
	Parameter<std::vector<std::string>>::Parameter(const std::string& name, const std::vector<std::string>& value):
	ParameterInterface(StateType::PARAMETER_STRING_ARRAY, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<CartesianState>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_CARTESIANSTATE, name)
	{}

	template <>
	Parameter<CartesianState>::Parameter(const std::string& name, const CartesianState& value):
	ParameterInterface(StateType::PARAMETER_CARTESIANSTATE, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<CartesianPose>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_CARTESIANPOSE, name)
	{}

	template <>
	Parameter<CartesianPose>::Parameter(const std::string& name, const CartesianPose& value):
	ParameterInterface(StateType::PARAMETER_CARTESIANPOSE, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<JointState>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_JOINTSTATE, name)
	{}

	template <>
	Parameter<JointState>::Parameter(const std::string& name, const JointState& value):
	ParameterInterface(StateType::PARAMETER_JOINTSTATE, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<JointPositions>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_JOINTPOSITIONS, name)
	{}

	template <>
	Parameter<JointPositions>::Parameter(const std::string& name, const JointPositions& value):
	ParameterInterface(StateType::PARAMETER_JOINTPOSITIONS, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<Ellipsoid>::Parameter(const std::string& name, const Ellipsoid& value):
	ParameterInterface(StateType::PARAMETER_ELLIPSOID, name), value(value)
	{
		this->set_filled();
	}

	template <>
	Parameter<Eigen::MatrixXd>::Parameter(const std::string& name):
	ParameterInterface(StateType::PARAMETER_MATRIX, name)
	{}

	template <>
	Parameter<Eigen::MatrixXd>::Parameter(const std::string& name, const Eigen::MatrixXd& value):
	ParameterInterface(StateType::PARAMETER_MATRIX, name), value(value)
	{
		this->set_filled();
	}

	template <typename T>
	std::ostream& operator<<(std::ostream& os, const Parameter<T>& parameter)
	{
		if(parameter.is_empty())
		{
			os << " Parameter " << parameter.get_name() << " is empty" << std::endl;
		}
		else
		{
			os << " Parameter " << parameter.get_name() << ": " << parameter.value << std::endl;
		}
  		return os;
	}

	template std::ostream& operator<<(std::ostream& os, const Parameter<double>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<bool>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<std::string>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<CartesianState>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<CartesianPose>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<JointState>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<JointPositions>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<Ellipsoid>& parameter);
	template std::ostream& operator<<(std::ostream& os, const Parameter<Eigen::MatrixXd>& parameter);

	template<>
	std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<double>>& parameter)
	{
		if(parameter.is_empty())
		{
			os << " Parameter " << parameter.get_name() << " is empty" << std::endl;
		}
		else
		{
			os << " Parameter " << parameter.get_name() << ": ";
			for (auto& v: parameter.value)
			{
				os << v << " | ";	
			}
			os << std::endl;
		}
  		return os;
	}

	template<>
	std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<bool>>& parameter)
	{
		if(parameter.is_empty())
		{
			os << " Parameter " << parameter.get_name() << " is empty" << std::endl;
		}
		else
		{
			os << " Parameter " << parameter.get_name() << ": ";
			for (auto v: parameter.value)
			{
				os << v << " | ";	
			}
			os << std::endl;
		}
  		return os;
	}

	template<>
	std::ostream& operator<<(std::ostream& os, const Parameter<std::vector<std::string>>& parameter)
	{
		if(parameter.is_empty())
		{
			os << " Parameter " << parameter.get_name() << " is empty" << std::endl;
		}
		else
		{
			os << " Parameter " << parameter.get_name() << ": ";
			for (auto& v: parameter.value)
			{
				os << v << " | ";	
			}
			os << std::endl;
		}
  		return os;
	}
}