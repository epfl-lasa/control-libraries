#pragma once

#include "clproto.h"

namespace clproto {

class DecoderNotImplementedException : public DecodingException {
public:
  explicit DecoderNotImplementedException(const std::string& msg);
};

/**
 * @brief Decoding helper method
 * @tparam ObjT The control libraries output type
 * @tparam MsgT The protocol message input type
 * @param message The protocol message object
 * @return The equivalent decoded control libraries object
 */
template<typename ObjT, typename MsgT>
ObjT decoder(const MsgT& message);

template<typename ObjT, typename MsgT>
ObjT decoder(const MsgT&) {
  throw DecoderNotImplementedException("Templated decoder function not implemented!");
}

}