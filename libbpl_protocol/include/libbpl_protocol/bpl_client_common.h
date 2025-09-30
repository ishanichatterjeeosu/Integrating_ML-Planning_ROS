#pragma once

#include "libbpl_protocol/packet.h"

namespace libbpl_protocol {

using libbpl_protocol::Packet;

/// \brief Functions common to to sync and async clients
///
/// Contains helpers to generate and send packets, it calls 
/// the virtual `send` function provided by either SyncClient
/// or AsynchronousClient.  These functions should **not** expect
/// a response from the actuator.
class BplClientCommon {
public:
  virtual void send(const Packet &packet) = 0;

  // == Various convenience functions

  /// Send a position command to an actuator.  Position is
  /// in rad for angular joints and mm for linear joints.
  void sendPosition(uint8_t device_id, float position);

  /// Send a velocity command to an actuator.  Velocity is in
  /// rad/sec for angular joints and mm/sec for linear joints.
  void sendVelocity(uint8_t device_id, float velocity);

  /// Broadcasts "set to mode MODE_STANDBY" to all actuators
  void enable();

  /// Broadcasts "set to mode MODE_DISABLE" to all actuators
  void disable();
};

} // namespace libbpl_protocol
