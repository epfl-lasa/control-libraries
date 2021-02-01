#pragma once

#include <iostream>
#include <exception>

namespace DynamicalSystems
{
	namespace Exceptions
	{
		class IncompatibleSizeException : public std::runtime_error
		{
		public:
			explicit IncompatibleSizeException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
