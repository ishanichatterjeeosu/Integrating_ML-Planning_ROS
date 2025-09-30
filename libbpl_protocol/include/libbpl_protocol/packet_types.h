//
// Copyright (c) 2022, University of Washington
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace libbpl_protocol {
typedef std::vector<uint8_t> ByteVector;

/// All packets end in the delimiter.  Due to COBS, the delimiter
/// never appears in the body of the packets.
const uint8_t Delimiter = 0x00;

/// Device address to broadcast to all devices.
const uint8_t BroadcastDeviceId = 0xFF;

/// Mode types, for use in MODE Packets
enum Mode_t {
  MODE_STANDBY = 0x00,
  MODE_DISABLE = 0x01,
  MODE_POSITION = 0x02,
  MODE_VELOCITY = 0x03,
  MODE_CURRENT = 0x04
};

/// Packet Types.   Taken from the BPL Protocol Documentation
struct PacketTypes {
  enum Type_t {
    // Rx == Message sent FROM actuator to PC
    // Tx == Message sent FROM PC to actuator
    // Tx/Rx == Message can be sent in either direction

    // (Tx/Rx) 1 byte:  Describe the current mode of the device
    //    Value | Mode
    //   -------|------
    //      0   | Standby
    //      1   | Disable
    //      2   | Position
    //      3   | Velocity
    //      4   | Current
    //
    // When in standby (and the other command modes?),
    // sending a control command will automatically switch to
    // that mode of operation.
    //
    // When disabled all other control commands will be ignored
    MODE = 0x01,

    // (Tx/Rx) 1 float: Set/get joint velocity, rad/sec for angular joints,
    // mm/sec for linear
    // If commanded velocity is greater than the max, it is set to the max
    // Returns the instantaneous velocity of the device
    VELOCITY = 0x02,

    // (Tx/Rx) 1 float: Set/get joint position, rad or mm
    // Angular position is bounded to 0 to 2PI
    POSITION = 0x03,

    // (Tx/Rx) 1 float: Set/get joint current draw in mA
    CURRENT = 0x05,

    // (Tx) 1 float:  Command position relative to current location
    RELATIVE_POSITION = 0x0E,

    //
    INDEXED_POSITION = 0x0D,

    //
    POSITION_LIMITS = 0x10,

    //
    VELOCITY_LIMITS = 0x11,

    // (Tx/Rx) 2 float: Configure maximum and minimum current in mA. Minimum current
    // is negative and refers to the maximum current in the negative direction. Does 
    // not override the factor limits.
    CURRENT_LIMITS = 0x12,

    // (Tx)  1-10 bytes : Request value(s) from the device.  Payload values are PacketTypes
    REQUEST = 0x60,

    // (Rx) 1 float : Serial numer as a four digit float
    SERIAL_NUMBER = 0x61,

    // (Rx) 1 float : Model number as a four digit number
    MODEL_NUMBER = 0x62,

    // (Rx) 1 float : Returns the inernal temp of the device in deg C
    TEMPERATURE = 0x66,

    // (Rx) 3 bytes : Returns the software version as three digits:
    // major.submajor.minor
    SOFTWARE_VERSION = 0x6C,

    // (rx) 1 float : Actuator supply voltage in V
    VOLTAGE = 0x90,

    // (Tx) 1-10 bytes : Sets the packet types included in the heartbeat message
    HEARTBEAT_SET = 0x91,

    // (Tx) 1 byte : Sets the frequency of heartbeat messages (1-255), or 0 to disable heartbeat messages
    HEARTBEAT_FREQUENCY = 0x92,

    //
    ATI_FT_READING = 0xD8
  };

  /// Convenience function which provides a string representation of the packet types.
  static std::string to_string(Type_t type);
};

} // namespace libbpl_protocol