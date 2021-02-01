#pragma once

#include <iostream>
#include <exception>

namespace DynamicalSystems
{
	namespace Exceptions
	{
		class NotImplementedException : public std::logic_error
		{
		public:
			explicit NotImplementedException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
