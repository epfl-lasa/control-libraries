

#pragma once

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleSizeException: public std::logic_error
		{
		public:
			explicit IncompatibleSizeException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
