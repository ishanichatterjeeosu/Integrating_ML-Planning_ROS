#!/usr/bin/env python3

## Make this script fail-safe if bplprotocol isn't installed
## If not, the tests will build with the version of autogen_test_data.h
## checked into git
##
try:
    from bplprotocol import BPLProtocol, PacketID
except ImportError:
    exit(0)

def testdata_out(out, packet_id, device_id, data, desc):
    buffer = BPLProtocol.encode_packet(device_id, packet_id, data)

    data_as_string = [("0x%02x" % d) for d in data]
    buffer_as_string = [("0x%02x" % d) for d in buffer]

    out.write("  BPLTestData(0x%02x, 0x%02x, {%s}, {%s}, \"%s\"),\n" % (packet_id, device_id, 
                                ','.join(data_as_string),
                                ','.join(buffer_as_string), desc))


if __name__ == "__main__":

    output_filename = "autogen_test_data.h"

    with open(output_filename,'w') as out:

        ## Print header
        out.write("#include \"test_data.h\"\n\n")
        out.write("std::vector<BPLTestData> testData = {\n")

        testdata_out(out, 0x00, 0x00, [], "Empty packet")
        testdata_out(out, PacketID.REQUEST, 0x01, bytes([PacketID.POSITION]), "REQUEST for POSITION to device 0x01")
        testdata_out(out, PacketID.POSITION, 0x0A, BPLProtocol.encode_floats([0.5]), "POSITION 0.5 to device 0x0A")


        # ## Generate test packets
        # device_id = 0x01  # Jaws
        # buffer = BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes([PacketID.POSITION]))

        # x = [hex(a) for a in buffer]
        # print(buffer)
        # print(x)


        ## And footer
        out.write("};\n")