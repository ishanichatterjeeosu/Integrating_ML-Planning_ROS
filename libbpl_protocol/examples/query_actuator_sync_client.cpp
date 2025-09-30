#include "libbpl_protocol/sync_client.h"
#include <iostream>

using libbpl_protocol::PacketTypes;

int main(int argc, char **argv) {

  const std::string ipAddress("192.168.2.3"); // Default IP address for Bravo
  const unsigned int port = 6789;             // Default port for Bravo

  libbpl_protocol::SyncClient client(ipAddress, port);

  const uint8_t deviceId = 0x01;

  auto p = client.queryFloat(PacketTypes::SERIAL_NUMBER, deviceId);
  if (p) {
    std::cout << "Device: " << static_cast<int>(deviceId)
              << " :: Serial number = " << p.value() << std::endl;
  }

  p = client.position(deviceId);
  if (p) {
    std::cout << "Device: " << static_cast<int>(deviceId)
              << " :: Position =  " << p.value() << std::endl;
  }

  p = client.velocity(deviceId);
  if (p) {
    std::cout << "Device: " << static_cast<int>(deviceId)
              << " :: Velocity =  " << p.value() << std::endl;
  }

  p = client.current(deviceId);
  if (p) {
    std::cout << "Device: " << static_cast<int>(deviceId)
              << " ::  Current =  " << p.value() << std::endl;
  }
  return 0;
}