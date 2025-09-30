#include <boost/asio.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace boost::asio;
using ip::udp;
using std::cout;
using std::endl;
using std::string;

#include "libbpl_protocol/async_client.h"
#include "libbpl_protocol/expected.hpp"
#include "libbpl_protocol/io_service_thread.h"
#include "libbpl_protocol/packet.h"

using libbpl_protocol::AsynchronousClient;
using libbpl_protocol::Packet;
using libbpl_protocol::PacketTypes;

typedef tl::expected<libbpl_protocol::Packet, std::string> PacketOrError;

int main(int argc, char **argv) {
  const string ipAddress("192.168.2.3"); // Default IP address for Bravo
  const unsigned int port = 6789;        // Default port for Bravo

  libbpl_protocol::IoServiceThread io_thread;
  AsynchronousClient bpl(io_thread.context(), ipAddress, port);
  std::chrono::time_point<std::chrono::steady_clock> time_last_send_;

  bpl.add_callback([&](Packet packet) {
    std::chrono::duration<double> dt =
        std::chrono::steady_clock::now() - time_last_send_;
    std::cout << "[ t = " << dt.count() << "] Device "
              << static_cast<int>(packet.deviceId()) << " packet type "
              << PacketTypes::to_string(packet.type()) << " (" << packet.type()
              << ")" << std::endl;
  });

  io_thread.start();

  // Turn off any existing heartbeat messages
  {
    Packet sent(PacketTypes::HEARTBEAT_FREQUENCY, 0xFF);
    sent.push_back<uint8_t>(0);
    bpl.send(sent);
  }

  if (false) {
    // request multiple values from client
    const uint8_t deviceId = 0xFF;
    Packet sent(PacketTypes::REQUEST, deviceId);
    sent.push_back(PacketTypes::POSITION);
    sent.push_back(PacketTypes::VELOCITY);
    sent.push_back(PacketTypes::CURRENT);
    bpl.send(sent);
  } else if (true) {
    // Try the heartbeat functionality
    const uint8_t deviceId = 0x07;
    {
      Packet sent(PacketTypes::HEARTBEAT_SET, deviceId);
      sent.push_back(PacketTypes::POSITION);
      sent.push_back(PacketTypes::VELOCITY);
      sent.push_back(PacketTypes::CURRENT);
      time_last_send_ = std::chrono::steady_clock::now();
      bpl.send(sent);

      // sent.setDeviceId(0x06);
      // bpl.send(sent);
    }
    {
      const uint8_t freq = 50; // Hz
      Packet sent(PacketTypes::HEARTBEAT_FREQUENCY, deviceId);
      sent.push_back<uint8_t>(freq);
      bpl.send(sent);
    }
  }

  // Just wait
  sleep(2);

  io_thread.stop();
  io_thread.join();

  // Turn off any existing heartbeat messages
  {
    Packet sent(PacketTypes::HEARTBEAT_FREQUENCY,
                libbpl_protocol::BroadcastDeviceId);
    sent.push_back<uint8_t>(0);
    bpl.send(sent);
  }

  bpl.disable();

  return 0;
}