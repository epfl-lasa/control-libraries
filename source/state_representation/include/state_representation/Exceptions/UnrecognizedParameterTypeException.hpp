#pragma once

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class UnrecognizedParameterTypeException: public std::logic_error
		{
		public:
			explicit UnrecognizedParameterTypeException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
