#pragma once

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleStatesException: public std::logic_error
		{
		public:
			explicit IncompatibleStatesException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
