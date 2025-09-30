#pragma once

#include "libbpl_protocol/packet_types.h"

libbpl_protocol::ByteVector cobsEncode(const libbpl_protocol::ByteVector &in);
libbpl_protocol::ByteVector cobsDecode(const libbpl_protocol::ByteVector &in);
