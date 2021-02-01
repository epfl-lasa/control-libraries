

#pragma once

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class NoSolutionToFitException: public std::runtime_error
		{
		public:
			explicit NoSolutionToFitException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
