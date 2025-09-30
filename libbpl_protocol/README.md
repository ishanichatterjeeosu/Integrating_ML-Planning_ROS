# libbpl_protocol

C++ library which implements the [Blueprint Lab](https://blueprintlab.com) [Reach Sysem](https://blueprintlab.com/products/manipulators/) communication protocol as described in their [Communication Protocol](https://blueprint-lab.github.io/Blueprint_Lab_Software/documentation/index.html).

This is essentially a C++ re-implementation of their [Python-only protocol library](https://github.com/blueprint-lab/Blueprint_Lab_Software).  

* Based on protocol document V1.11.0
* Only implements the open "Lite" protocol, which does not include the inverse kinematics functionality etc of their "Pro" package.

## Dependencies

This package is build around the [catkin]() packaging tool, since we're targeting ROS, but this package itself has no ROS dependencies.

* `boost::asio` is required to build the examples

# Quick API tour

A [`Packet`](include/libbpl_protocol/packet.h) is a single BPL packet.  It contains three data: a [packet type](include/libbpl_protocol/packet_types.h), a device id, and an array of bytes containing the payload for the packet.  It has functions for reading and writing data values to the payload, but the Packet doesn't understand the meaning of the bytes in the payload (e.g. is it a float or four 8-bit ints?).

The `encode()` member function writes the Packet contents to a `vector<uint8_t>`, adding the checksum and performing the [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) encoding.

The `Packet::Decode()` static function converts a buffer to a Packet.


```
const uint8_t DeviceId = 0x01;
Packet packet(PacketTypes::VELOCITY, DeviceId);
packet.push_back<float>(0.5);

vector<uint8_t> buffer = packet.encode();

// (...write buffer to device...)
// (...receive from device...)

auto received = Packet::Decode(buffer);

// Received is an expected<> which could be either a Packet or an error type.

if (received) {
  // Note pop_front is destructive, it deletes the returned bytes
  // from the buffer

  float velocity = received.pop_front<float>();  // should == 0.5
}
```


# Example applications

Example applications are in the [`examples`](examples/) directory:

* [`examples/query_actuator_sync`](examples/query_actuator_sync.cpp) : Performs a synchronous query of an actuator
* [`examples/query_actuator_sync_client`](examples/query_actuator_sync_client.cpp) : Performs a synchronous query of an actuator using the [sync_client](src/sync_client.cpp) class.
* [`examples/query_actuator_async`](examples/query_actuator_async.cpp) : Queries the actuator using the [AsynchronousClient](src/async_client.cpp).


# Testing

This repo contains a brief test suite which automatically compares outputs from the C++ library with packets generated using the reference Python library.   `catkin build && catkin test` to run.

# TODO

* Higher level abstraction for Packet types (e.g. VelocityPacket) which understands the data in the buffer.
* Add support for serial communications

# License

Released under the BSD 3-Clause license.   This software is based on publicly available SDK and API documentation from Blueprint Lab and contains no code written by Blueprint Lab.  We do try to align our constants to agree with those used in Python library.
