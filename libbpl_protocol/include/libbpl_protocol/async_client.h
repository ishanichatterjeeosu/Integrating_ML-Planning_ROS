#pragma once

#include <boost/asio.hpp>
#include <functional>

#include "libbpl_protocol/bpl_client_common.h"
#include "libbpl_protocol/expected.hpp"
#include "libbpl_protocol/packet.h"
#include "libbpl_protocol/packet_types.h"

namespace libbpl_protocol {

using boost::asio::ip::udp;

class AsynchronousClient : public BplClientCommon {
public:
  AsynchronousClient(boost::asio::io_service &service,
                     const std::string ipAddress, const int port);

  void readNext();

  void send(const libbpl_protocol::Packet &v) override;

  void onReceive(const boost::system::error_code &ec,
                 std::size_t bytes_transferred);

  typedef std::function<void(Packet)> PacketCallbackFunc;
  void add_callback(PacketCallbackFunc callback) {
    packetCallbacks_.push_back(callback);
  }

private:
  udp::socket socket_;

  libbpl_protocol::ByteVector buffer_;

  std::vector<PacketCallbackFunc> packetCallbacks_;
};

} // namespace libbpl_protocol