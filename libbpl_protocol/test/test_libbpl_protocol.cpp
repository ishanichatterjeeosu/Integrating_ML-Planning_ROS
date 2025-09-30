#include "gtest/gtest.h"
#include "autogen_test_data.h"
#include "libbpl_protocol/packet.h"

using libbpl_protocol::ByteVector;
using libbpl_protocol::Packet;
using libbpl_protocol::PacketTypes;

TEST(PacketTest, TestConstructor) {
  const uint8_t dev_id = 0x01;
  Packet packet(PacketTypes::MODE, dev_id);

  ASSERT_EQ(packet.type(), PacketTypes::MODE);
  ASSERT_EQ(packet.deviceId(), dev_id);
}

TEST(PacketTest, TestTestData) {
  for (auto const &test : testData) {
    Packet packet(static_cast<PacketTypes::Type_t>(test.packet_type),
                  test.device_id, test.data);

    ByteVector buffer = packet.encode();

    ASSERT_EQ(buffer, test.buffer)
        << "Packet output does not match test data for \"" << test.description
        << "\"";

    // And decode buffer
    {
      auto d = Packet::Decode(test.buffer);

      // Assert that it's not an error
      ASSERT_TRUE(d);

      if (d) {
        auto decoded = d.value();
        ASSERT_EQ(packet.type(), decoded.type());
        ASSERT_EQ(packet.deviceId(), decoded.deviceId());
        ASSERT_EQ(packet.data(), decoded.data());
      }
    }
  }
}

// Hand crafted tests
TEST(PacketTest, HandCraftedPackets) {
  {
    BPLTestData TestPacket(
        0x03, 0x0a, {0x00, 0x00, 0x00, 0x3f},
        {0x01, 0x01, 0x01, 0x06, 0x3f, 0x03, 0x0a, 0x08, 0x1e, 0x00},
        "POSITION 0.5 to device 0x0A");

    const auto vel = 0.5;
    Packet packet(PacketTypes::POSITION, 0x0A);
    packet.push_back<float>(vel);

    auto buffer = packet.encode();

    ASSERT_EQ(buffer, TestPacket.buffer);

    ASSERT_EQ(packet.pop_front<float>(), vel);
  }
}
