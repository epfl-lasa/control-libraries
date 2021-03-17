#pragma once

#include <iostream>
#include <exception>

namespace dynamical_systems
{
	namespace exceptions
	{
		class NotImplementedException : public std::logic_error
		{
		public:
			explicit NotImplementedException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
