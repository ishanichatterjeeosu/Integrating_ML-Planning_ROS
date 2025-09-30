

#include "libbpl_protocol/bpl_client_common.h"

namespace libbpl_protocol {

void BplClientCommon::sendPosition(uint8_t device_id, float position) {
  Packet sent(PacketTypes::POSITION, device_id);
  sent.push_back<float>(position);
  send(sent);
}

void BplClientCommon::sendVelocity(uint8_t device_id, float velocity) {
  Packet sent(PacketTypes::VELOCITY, device_id);
  sent.push_back<float>(velocity);
  send(sent);
}

// Broadcasts "set to mode MODE_STANDBY" to all actuators
// void BplClientCommon::enable() {
//   Packet sent(PacketTypes::MODE, libbpl_protocol::BroadcastDeviceId);
//   sent.push_back<uint8_t>(MODE_STANDBY);
//   send(sent);
// }
void BplClientCommon::enable() {
  Packet sent(PacketTypes::MODE, libbpl_protocol::BroadcastDeviceId);
  sent.push_back<uint8_t>(MODE_STANDBY);
  send(sent);
}

// Broadcasts "set to mode MODE_DISABLE" to all actuators
void BplClientCommon::disable() {
  Packet sent(PacketTypes::MODE, libbpl_protocol::BroadcastDeviceId);
  sent.push_back<uint8_t>(MODE_DISABLE);
  send(sent);
}

} // namespace libbpl_protocol