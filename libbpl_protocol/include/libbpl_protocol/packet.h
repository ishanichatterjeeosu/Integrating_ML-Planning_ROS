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

#include <vector>

#include "libbpl_protocol/bpl_error.h"
#include "libbpl_protocol/packet_types.h"

namespace libbpl_protocol {

///
/// From the BPL documentation, the packet structure is as follows:
///
///  Byte(s)      | Meaning
///  -------------|------------
///  1            | COBS Bytes
///  1:(Length-4) | Data
///  Length-3     | Packet_id
///  Length-2     | Device_id
///  Length-1     | Length
///  Length       | CRC
///               | Terminator == 0x00
///
/// The COBS byte is an artifact of the COBS stuffing
///  https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
///
/// The data fields are packed into the packet, with the
///  quantity, data type and meaning determined by the packet_id
///  Packets cannot be any longer than 254 bytes
///
/// Length is the total packet length including the 4-byt "footer":
///  packet_id, device_id, length and CRC
///
/// CRC is an 8-bit CRC over the whole packet except the terminator
///
/// The terminator byte is always 0x00.  Due to the COBS encoding,
///  this is the **only** 0x00 in the byte stream.
///
class Packet {
public:
  typedef uint8_t DeviceId_t;

  Packet() = delete;

  /// Default copy constructor
  Packet(const Packet &) = default;

  /// Constructor to initialize Packet with a given Packet type, device id, and optionally a data buffer.
  Packet(PacketTypes::Type_t tp, DeviceId_t device, ByteVector data = {});

  /// Attempts to read a Packet from ByteVector
  /// Returns an expected<> which contains either the Packet or a BplError
  /// if there is a parsing error.
  static PacketOrError Decode(const ByteVector &buffer);

  /// PacketType of the packet
  PacketTypes::Type_t type() const { return _type; }

  /// Devie id of the packet
  DeviceId_t deviceId() const { return _device; }

  /// Raw byte vector of the packet
  ByteVector data() const { return _data; }

  /// Set the device id in the packet
  Packet &setDeviceId(uint8_t d) {
    _device = d;
    return *this;
  }

  /// Set the PacketType
  Packet &setType(PacketTypes::Type_t tp) {
    _type = tp;
    return *this;
  }

  /// Push a value of type Tp to the end of the buffer.
  template <typename Tp> Packet &push_back(Tp t) {
    const uint8_t *b = reinterpret_cast<const uint8_t *>(&t);

    for (size_t i = 0; i < sizeof(Tp); ++i) {
      _data.push_back(b[i]);
    }

    return *this;
  }

  /// Push a two values of type Tp to the end of the buffer
  template <typename Tp> Packet &push_back(Tp a, Tp b) {
    push_back(a);
    return push_back(b);
  }

  /// Push a three values of type Tp to the end of the buffer
  template <typename Tp> Packet &push_back(Tp a, Tp b, Tp c) {
    push_back(a);
    push_back(b);
    return push_back(c);
  }

  /// Push a four values of type Tp to the end of the buffer
  template <typename Tp> Packet &push_back(Tp a, Tp b, Tp c, Tp d) {
    push_back(a);
    push_back(b);
    push_back(c);
    return push_back(d);
  }

  /// Pop a value of type Tp off the front of the buffer.
  /// Note this is a destructive operation, removing data from the buffer
  /// Packet has no knowledge of the contents or framing of the buffer, it 
  /// is up to the end user to understand the data associated with a given
  /// PacketType
  template <typename Tp> Tp pop_front() {
    Tp val = *reinterpret_cast<Tp *>(_data.data());
    _data.erase(_data.begin(), _data.begin() + sizeof(Tp));
    return val;
  }

  /// Encode the Packet to a ByteVector.
  /// This includes formatting the type, device id and buffer data,
  /// calculating the CRC and performing COBS encoding.
  ByteVector encode() const;

protected:
  PacketTypes::Type_t _type;
  DeviceId_t _device;
  ByteVector _data;
};

} // namespace libbpl_protocol