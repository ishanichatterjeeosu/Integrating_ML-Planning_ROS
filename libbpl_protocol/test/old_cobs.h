#pragma once

#include "libbpl_protocol/packet_types.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t old_cobsEncode(const void *data, size_t length, uint8_t *buffer);
size_t old_cobsDecode(const uint8_t *buffer, size_t length, void *data);

#ifdef __cplusplus
}

#endif
