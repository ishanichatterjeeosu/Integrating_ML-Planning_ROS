#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>

using namespace boost::asio;
using ip::udp;
using std::cout;
using std::endl;
using std::string;

#include "libbpl_protocol/expected.hpp"
#include "libbpl_protocol/packet.h"
#include "libbpl_protocol/bpl_error.h"

using libbpl_protocol::Packet;
using libbpl_protocol::PacketTypes;
using libbpl_protocol::PacketOrError;

// Note this function waits for **ONE** packet in return
// it will not check for additional packets.
//
// \todo No clean way to report failure
// \todo No timeout
PacketOrError send_receive(udp::socket &socket,
                           const libbpl_protocol::Packet &packet) {

  libbpl_protocol::ByteVector out(packet.encode());

  auto len = socket.send(boost::asio::buffer(out));
  std::cerr << "Sent " << len << " bytes to arm" << std::endl;

  libbpl_protocol::ByteVector v(256);
  len = socket.receive(boost::asio::buffer(v));
  v.resize(len);

  std::cout << "Received " << v.size() << " bytes from arm" << std::endl;

  return Packet::Decode(v);
}

int main(int argc, char **argv) {

  const string ipAddress("192.168.2.3"); // Default IP address for Bravo
  const unsigned int port = 6789;        // Default port for Bravo
  boost::asio::io_context context;

  udp::socket socket(context);
  socket.connect(
      udp::endpoint(boost::asio::ip::address::from_string(ipAddress), port));

  // Build packet to request serial number from client
  const uint8_t deviceId = 0x01;
  Packet sent_packet(PacketTypes::REQUEST, deviceId);
  sent_packet.push_back(PacketTypes::SERIAL_NUMBER);

  auto received = send_receive(socket, sent_packet);

  if (received) {
    auto rx_packet = received.value();
    if (rx_packet.type() == PacketTypes::SERIAL_NUMBER) {
      // Serial number is encoded as a single float
      std::cout << "Serial number for id " << static_cast<int>(deviceId)
                << " is " << rx_packet.pop_front<float>() << std::endl;
    } else {
      std::cerr << "Got unexpected packet type " << std::hex << rx_packet.type()
                << std::endl;
    }
  } else {
    std::cout << "Error: " << received.error().msg() << std::endl;
  }

  return 0;
}