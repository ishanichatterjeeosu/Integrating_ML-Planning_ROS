#include <iostream>
#include <iomanip>
#include "libbpl_protocol/async_client.h"

namespace libbpl_protocol {

AsynchronousClient::AsynchronousClient(boost::asio::io_service &service,
                                       const std::string ipAddress,
                                       const int port)
    : socket_(service) {
  socket_.connect(
      udp::endpoint(boost::asio::ip::address::from_string(ipAddress), port));

  readNext();
}

void AsynchronousClient::readNext() {
  buffer_.resize(256);
  socket_.async_receive(boost::asio::buffer(buffer_),
                        std::bind(&AsynchronousClient::onReceive, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
}

void AsynchronousClient::send(const libbpl_protocol::Packet &v) {
  boost::system::error_code error;

  // std::cout << "id: " << (int) v.deviceId() << std::endl;
  // std::cout << "type: 0x" << std::setw(2) << std::setfill('0') << std::hex << (v.type() & 0xff) << std::endl;

  auto data = v.data();

  // for (auto &dat : data)
  // {
  //   std::cout << "data: 0x" << std::setw(2) << std::setfill('0') << std::hex << (dat & 0xff) << std::endl;
  // }

  auto enc = v.encode();

  // for (auto &en : enc)
  // {
  //   std::cout << "encode: 0x" << std::setw(2) << std::setfill('0') << std::hex << (en & 0xff) << std::endl;
  // }

  // std::cout << "\n"  << std::endl;


  auto len = socket_.send(boost::asio::buffer(v.encode()));
  
  
  //std::cout << "encode" << v.encode() << std::endl;

  // \todo Ignoring any errors for now
  // std::cerr << "[ t = " << 0.0 << "] Sent " << len << " bytes" << std::endl;
  // Adding more debug statements
  // std::cout << "[ t = " << 0.0 << "] Sent " << len << " bytes" << std::endl;

}

void AsynchronousClient::onReceive(const boost::system::error_code &ec,
                                   std::size_t bytes_transferred) {
  if (ec) {
    // On error, just abandon and wait for the next packet
    readNext();
  }

  // std::cout << "[ t = " << dt.count() << "] Received " << bytes_transferred << " bytes" << std::endl;
  buffer_.resize(bytes_transferred);

  auto p = Packet::Decode(buffer_);
  if (p) {
    auto packet = p.value();
    for (auto c : packetCallbacks_)
      c(packet);
  } else {
    std::cout << "Error: " << p.error().msg() << std::endl;
  }

  readNext();
}

} // namespace libbpl_protocol