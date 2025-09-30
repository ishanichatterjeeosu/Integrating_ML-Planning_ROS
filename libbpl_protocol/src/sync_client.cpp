
#include <iostream>

#include "libbpl_protocol/sync_client.h"

namespace libbpl_protocol {

SyncClient::SyncClient(const std::string &ipAddress, int port)
    : context_(), socket_(context_) {
  socket_.connect(
      udp::endpoint(boost::asio::ip::address::from_string(ipAddress), port));
}

PacketOrError SyncClient::send_receive(const libbpl_protocol::Packet &packet) {

  libbpl_protocol::ByteVector out(packet.encode());

  auto len = socket_.send(boost::asio::buffer(out));

  libbpl_protocol::ByteVector v(256);
  len = socket_.receive(boost::asio::buffer(v));
  v.resize(len);

  return Packet::Decode(v);
}

void SyncClient::send(const libbpl_protocol::Packet &packet) {
  // Only works with messages that don't return a response
  libbpl_protocol::ByteVector out(packet.encode());
  socket_.send(boost::asio::buffer(out));
}

FloatOrError SyncClient::queryFloat(PacketTypes::Type_t type,
                                    const uint8_t deviceId) {
  Packet sent_packet(PacketTypes::REQUEST, deviceId);
  sent_packet.push_back(type);

  auto received = send_receive(sent_packet);

  if (!received)
    return tl::make_unexpected(received.error());

  auto rx_packet = received.value();

  if (rx_packet.type() != type)
    return tl::make_unexpected(
        BplError(BPL_UNEXPECTED_TYPE, "Unexpected type"));

  return rx_packet.pop_front<float>();
}

} // namespace libbpl_protocol