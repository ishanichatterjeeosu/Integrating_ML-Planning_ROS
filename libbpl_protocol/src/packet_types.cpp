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

#include "libbpl_protocol/packet_types.h"

namespace libbpl_protocol {

std::string PacketTypes::to_string(Type_t type) {

  if (type == PacketTypes::MODE)
    return "MODE";
  else if (type == PacketTypes::VELOCITY)
    return "VELOCITY";
  else if (type == PacketTypes::POSITION)
    return "POSITION";
  else if (type == PacketTypes::CURRENT)
    return "CURRENT";
  else if (type == PacketTypes::RELATIVE_POSITION)
    return "RELATIVE_POSITION";
  else if (type == PacketTypes::INDEXED_POSITION)
    return "INDEXED_POSITION";
  else if (type == PacketTypes::VELOCITY_LIMITS)
    return "VELOCITY_LIMITS";
  else if (type == PacketTypes::CURRENT_LIMITS)
    return "CURRENT_LIMITS";
  else if (type == PacketTypes::REQUEST)
    return "REQUESTS";
  else if (type == PacketTypes::SERIAL_NUMBER)
    return "SERIAL_NUMBER";
  else if (type == PacketTypes::MODEL_NUMBER)
    return "MODEL_NUMBER";
  else if (type == PacketTypes::TEMPERATURE)
    return "TEMPERATURE";
  else if (type == PacketTypes::SOFTWARE_VERSION)
    return "SOFTWARE_VERSION";
  else if (type == PacketTypes::VOLTAGE)
    return "VOLTAGE";
  else if (type == PacketTypes::HEARTBEAT_SET)
    return "HEARTBEAT_SET";
  else if (type == PacketTypes::HEARTBEAT_FREQUENCY)
    return "HEARTBEAT_FREQUENCY";
  else if (type == PacketTypes::ATI_FT_READING)
    return "ATI_FT_READING";

  return "(unknown)";
}

}; // namespace libbpl_protocol
