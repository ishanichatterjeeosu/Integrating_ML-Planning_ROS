#pragma once

#include <boost/asio.hpp>

#include "libbpl_protocol/bpl_client_common.h"
#include "libbpl_protocol/expected.hpp"
#include "libbpl_protocol/packet.h"
#include "libbpl_protocol/packet_types.h"

namespace libbpl_protocol {

using boost::asio::ip::udp;

using libbpl_protocol::Packet;

class SyncClient : public BplClientCommon {
public:
  SyncClient(const std::string &ipAddress = "192.168.2.3", int port = 6789);

  // Note this function waits for **ONE** packet in return
  // it will not check for additional packets.
  //
  // \todo No clean way to report failure
  // \todo No timeout
  PacketOrError send_receive(const Packet &packet);

  void send(const Packet &packet) override;

  // Simple accessors
  FloatOrError queryFloat(PacketTypes::Type_t type, const uint8_t deviceId);
  FloatOrError velocity(const uint8_t deviceId) {
    return queryFloat(PacketTypes::VELOCITY, deviceId);
  }
  FloatOrError position(const uint8_t deviceId) {
    return queryFloat(PacketTypes::POSITION, deviceId);
  }
  FloatOrError current(const uint8_t deviceId) {
    return queryFloat(PacketTypes::CURRENT, deviceId);
  }

private:
  boost::asio::io_context context_;
  udp::socket socket_;
};

} // namespace libbpl_protocol