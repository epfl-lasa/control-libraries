# clproto_cpp - Control Libraries Protocol (C++)

This library provides a simple interface for encoding and decoding
control library type objects to and from a serialized binary string
representation (the wire format).

```c++
#include <clproto.h>
#include <state_representation/space/cartesian/CartesianState.hpp>

auto state = state_representation::CartesianState::Random("frame");

// Encode the message into the wire format
std::string message = clproto::encode(state);

// The type is automatically derived from the object,
// but can also be provided explicitly
clproto::encode<state_representation::CartesianState>(state);

// Decode the message back into an object
auto new_state = clproto::decode<state_representation::CartesianState>(message);

// The previous method will throw an exception if the
// message cannot be parsed into that object type.
// For exception-safe decoding, pass an object by reference 
// to the decode function and check the bool return value
state_representation::CartesianState state_reference;
if (clproto::decode(message, state_reference)) {
  // successful decoding, state_reference has been modified
} else {
  // unsuccessful decoding, exception is suppressed and state_reference is unmodified
}

// If the message type is not known, use the following
// methods to check the validity and type
if (!clproto::is_valid(message)) {
  // message cannot be decoded into any known type
}

clproto::MessageType type = clproto::check_message_type(msg);

```

## Installation (TODO)

- How to build clproto and use it in an external project