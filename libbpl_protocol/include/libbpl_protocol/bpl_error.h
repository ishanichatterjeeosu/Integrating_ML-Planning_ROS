#pragma once

#include "libbpl_protocol/expected.hpp"

namespace libbpl_protocol {

class Packet;

enum BplError_t { BPL_BAD_PACKET, BPL_BAD_CRC, BPL_UNEXPECTED_TYPE };

class BplError {
public:
  BplError(BplError_t err, const std::string &msg = "")
      : err_(err), msg_(msg) {}

  const std::string &msg() const { return msg_; }
  BplError_t error() const { return err_; }

private:
  BplError_t err_;
  std::string msg_;
};

typedef tl::expected<Packet, BplError> PacketOrError;
typedef tl::expected<float, BplError> FloatOrError;

} // namespace libbpl_protocol