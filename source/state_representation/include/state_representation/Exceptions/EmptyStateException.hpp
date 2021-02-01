

#pragma once

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class EmptyStateException: public std::runtime_error
		{
		public:
			explicit EmptyStateException(const std::string& msg) : runtime_error(msg)
			{};
		};
	}
}
