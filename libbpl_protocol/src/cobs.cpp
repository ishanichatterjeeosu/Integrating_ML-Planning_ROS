#include <iostream>

#include "cobs.h"

using libbpl_protocol::ByteVector;

ByteVector cobsEncode(const ByteVector &const_in) {
  // Create working copy with 0x00 prepended to front of buffer
  ByteVector in = {0x00};
  in.insert(in.end(), const_in.begin(), const_in.end());

  ByteVector out;
  out.reserve(in.size());

  // We'll be bad and use indices not iterators
  for (size_t i = 0; i < in.size(); i++) {
    if (in[i] != 0x00) {
      // If non-zero just push to the output
      out.push_back(in[i]);
    } else {
      // Otherwise lookahead
      size_t j = i + 1;
      for (; j < in.size(); j++) {
        if (in[j] == 0x00) {
          out.push_back(j - i);
          break;
        }
      }

      // If you exited the loop without finding 0x00
      // push bytes to the end of the packet
      if (j >= in.size()) {
        out.push_back(in.size() - i);
      }
    }
  }

  return out;
}

libbpl_protocol::ByteVector
cobsDecode(const libbpl_protocol::ByteVector &const_in) {
  ByteVector in(const_in);

  size_t i = 0;
  for (size_t i = 0; i < in.size();) {
    const auto next = in[i];
    in[i] = 0x00;
    i += next;
  }

  // Take everything but the first byte
  ByteVector out(in.begin() + 1, in.end());
  return out;
}
