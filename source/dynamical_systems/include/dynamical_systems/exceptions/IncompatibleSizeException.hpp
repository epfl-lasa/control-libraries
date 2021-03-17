#pragma once

#include <iostream>
#include <exception>

namespace dynamical_systems
{
	namespace exceptions
	{
		class IncompatibleSizeException : public std::runtime_error
		{
		public:
			explicit IncompatibleSizeException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
