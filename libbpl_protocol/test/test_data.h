#pragma once

#include <stdint.h>

#include <string>
#include <vector>

class BPLTestData {
 public:
  BPLTestData(uint8_t p, uint8_t d, std::vector<uint8_t> dat,
              std::vector<uint8_t> buf, const std::string desc)
      : packet_type(p),
        device_id(d),
        data(dat),
        buffer(buf),
        description(desc) {}

  uint8_t packet_type;
  uint8_t device_id;

  std::vector<uint8_t> data;
  std::vector<uint8_t> buffer;
  std::string description;
};

// std::vector<BPLTestData> testData = {
//     BPLTestData(0x00, 0x00, {}, {0x05, 0x00, 0x00, 0x04, 0x00, 0x00}, "Sample
//     data")};
