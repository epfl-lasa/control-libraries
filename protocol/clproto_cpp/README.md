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

## Combining multiple message into a packet

The serialized binary string encoding of a state message is not self-delimiting.
The consequence of this is that when multiple encoded messages are combined on the wire,
it is not possible to know where the first message ends and the second one starts.

There will often be cases where it makes sense to combine multiple state messages 
into one high level message packet. For example, a robot might broadcast the `CartesianState`
of its end-effector together with its `JointState`. A client controller might want to listen
for the combined "Robot State" on a single subscription topic.

The clproto library facilitates this usage by providing the methods `pack_fields` and `unpack_fields`.
An ordered vector of encoded messages are combined into a data packet by the `pack_fields` method.
For N fields, it adds a message header with (N + 1) values, describing the number of fields
and the data size of each ordered field method. This allows the `unpack_fields` method to split
the encoded fields back into an ordered vector to be decoded.

```c++
#include <clproto.h>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/JointState.hpp>

using namespace state_representation;

// A robot can produce multiple state messages
auto cart_state = CartesianState::Random("robot_ee", "robot_base");
auto joint_state = JointState::Random("robot", 7);

// Encode each state variable and add them to an ordered vector
std::vector<std::string> encoded_robot_state;
encoded_robot_state.emplace_back(clproto::encode(cart_state));
encoded_robot_state.emplace_back(clproto::encode(joint_state));

// Pack the message fields into a raw data buffer,
// reserving sufficient space to contain all messages
char encoded_packet_buffer[2 * CLPROTO_PACKING_MAX_FIELD_LENGTH];
clproto::pack_fields(encoded_robot_state, encoded_packet_buffer);

// The message fields can also be packed into a std::string,
// provided that sufficient space is reserved for all messages
std::string encoded_packet_str();
encoded_packet_str.reserve(2 * CLPROTO_PACKING_MAX_FIELD_LENGTH);
clproto::pack_fields(encoded_robot_state, encoded_packet_str.data());

// Unpack a combined message packet back into an ordered vector of encoded state variables
std::vector<std::string> unpacked_encoded_robot_state = clproto::unpack(encoded_packet_buffer);
// (or, for a string type encoded packet:)
unpacked_encoded_robot_state = clproto::unpack(encoded_packet_str.c_str());

// Finally, decode the encoded fields in the same order as the original packing
auto decoded_cart_state = clproto::decode<CartesianState>(unpacked_encoded_robot_state.at(0));
auto decoded_joint_state = clproto::decode<JointState>(unpacked_encoded_robot_state.at(1));
```

The pack / unpack methods are provided as a convenience only for simple synthesis of messages
from core state message types. The communication implementation can always be extended further
by the end user as necessary, for example to prepend an indication of the field types to the message, 
or to add other delimiting behaviour.

### ZMQ example

The packing and unpacking of the message should be handled according to the network requirements.
This sections shows an example for ZMQ messaging.

```c++
#include <zmq.h>

// Combine encoded state messages into an ordered vector
std::vector<std::string> encoded_robot_state = ...;

// Pack and publish the combined state messages, assuming a pre-configured publishing socket
zmq::socket_t publisher;
zmq::message_t zmq_message(encoded_robot_state.size() * CLPROTO_PACKING_MAX_FIELD_LENGTH);
pack_fields(encoded_robot_state, static_cast<char *>(zmq_message.data()));
publisher.send(zmq_message, zmq::send_flags::none);

// On the receiving side, cast the message data to a char pointer before unpacking
zmq::message_t received_zmq_message;
zmq::socket_t subscriber;
auto result = subscriber.recv(received_zmq_message);
if (result) {
  received_encoded_robot_state = unpack_fields(static_cast<const char*>(received_zmq_message.data()));
}
```

### Reserving packet size
Note that, because the packing function writes to a buffer through a raw pointer, the associated
buffer must have enough reserved size to contain the entire packet. For simplicity, the
message size can be reserved as `number_of_fields * CLPROTO_PACKING_MAX_FIELD_LENGTH`, but
this is almost much larger than necessary. Since the resultant packed data is not self-delimiting,
the actual data length cannot be calculated post-hoc. 

If bandwidth is limited, the following formula can be used to calculate the minimum needed buffer size.

```c++
std::vector<std::string> encoded_fields = ...;

// The packet header starts with N + 1 values of size proto::field_length_t,
// where N is the number of fields N. The first value stores the number of fields,
// while the following N values store the data length of each respective field.
std::size_t packet_size = sizeof(proto::field_length_t) * (encoded_fields.size() + 1);

// Add the size of each field
for (const auto& field : encoded_fields) {
  packet_size += field.size();
}
```