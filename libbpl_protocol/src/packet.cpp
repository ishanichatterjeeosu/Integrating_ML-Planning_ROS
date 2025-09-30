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

#include "libbpl_protocol/packet.h"

#include <iostream>
#include <iterator>
#include <vector>

#include "cobs.h"
#include "crc.h"

namespace libbpl_protocol {

Packet::Packet(PacketTypes::Type_t tp, DeviceId_t device, ByteVector data)
    : _type(tp), _device(device), _data(data) {}

ByteVector Packet::encode() const {
  ByteVector out(_data);

  out.push_back(_type);
  out.push_back(_device);

  // Length is current buffer plus two (length and CRC)
  out.push_back(out.size() + 2);

  // Calculate CRC
  auto crc = crcSlow(out.data(), out.size());
  out.push_back(crc);

  // COBS encode
  auto cobs_out = cobsEncode(out);

  // Add terminator
  cobs_out.push_back(0x00);

  return cobs_out;
}

//==========

PacketOrError Packet::Decode(const ByteVector &buffer) {

  if (buffer.size() == 0)
    return tl::make_unexpected(BPL_BAD_PACKET);

  if (buffer.back() != libbpl_protocol::Delimiter)
    return tl::make_unexpected(BPL_BAD_PACKET);

  // Drop terminator
  ByteVector work(buffer);
  work.pop_back();

  work = cobsDecode(work);

  // Check CRC
  auto buffer_crc = work.back();
  work.pop_back();

  auto crc = crcSlow(work.data(), work.size());

  if (crc != buffer_crc) {
    std::cerr << "CRC mismatch" << std::endl;
    return tl::make_unexpected(BPL_BAD_CRC);
  }

  //   std::cout << "Buffer CRC = " << std::hex << static_cast<int>(buffer_crc)
  //             << " calculated = " << static_cast<int>(crc) << std::endl;

  auto len = work.back();
  work.pop_back();

  // Check length?

  uint8_t device_id = work.back();
  work.pop_back();
  uint8_t packet_type = work.back();
  work.pop_back();

  return Packet(static_cast<PacketTypes::Type_t>(packet_type), device_id, work);
}

// Specialization for PacketTypes
template <> Packet &Packet::push_back(PacketTypes::Type_t t) {
  return push_back<uint8_t>(static_cast<uint8_t>(t));
}

} // namespace libbpl_protocol