#include "gtest/gtest.h"

#include "cobs.h"
#include "old_cobs.h"

using libbpl_protocol::ByteVector;

std::vector<ByteVector> cobs_test_data = {
    {0x1, 0x2, 0x3, 0x4},
    {0x0, 0x1, 0x2, 0x3},
    {0x0, 0x1, 0x0, 0x2, 0x3, 0x0, 0x0, 0x5}};

TEST(CobsTest, Test) {
  for (const auto td : cobs_test_data) {
    ByteVector new_cobs = cobsEncode(td);

    uint8_t b[255];
    auto blen = old_cobsEncode(td.data(), td.size(), b);

    ByteVector old_cobs(reinterpret_cast<uint8_t *>(std::begin(b)),
                        reinterpret_cast<uint8_t *>(std::begin(b)) + blen);

    ASSERT_EQ(new_cobs, old_cobs);

    // And test decoding
    ByteVector new_decode = cobsDecode(old_cobs);

    uint8_t c[255];
    auto clen = old_cobsDecode(old_cobs.data(), old_cobs.size(), c);

    ByteVector old_decode(reinterpret_cast<uint8_t *>(std::begin(c)),
                          reinterpret_cast<uint8_t *>(std::begin(c)) + clen);

    ASSERT_EQ(new_decode, old_decode);
  }
}