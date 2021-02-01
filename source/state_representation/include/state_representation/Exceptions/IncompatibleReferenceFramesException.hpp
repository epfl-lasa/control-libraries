

#pragma once

#include <iostream>
#include <exception>

namespace StateRepresentation
{
	namespace Exceptions
	{
		class IncompatibleReferenceFramesException: public std::logic_error
		{
		public:
			explicit IncompatibleReferenceFramesException(const std::string& msg) : logic_error(msg)
			{};
		};
	}
}
