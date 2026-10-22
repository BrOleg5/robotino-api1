#!/usr/bin/env python3
"""Extracts OpenRobotinoAPI 1 protocol messages from a pcapng capture.

Reassembles the TCP streams to and from the robot port (80 by default), splits them into
messages ([id: u8][length: u16 LE][payload]) and either lists them or writes selected
messages to files. Only the Python standard library is used.

Examples:
  pcap_extract.py capture.pcapng
  pcap_extract.py capture.pcapng --write tests/data --select iocontrol:0 iostatus:0 info:0
"""

import argparse
import pathlib
import struct
import sys

MESSAGE_NAMES = {
    0: "iocontrol",
    1: "iostatus",
    2: "cameracontrol",
    3: "keepalive",
    4: "rawdata",
    5: "connectionbusy",
    6: "info",
}

LINKTYPE_ETHERNET = 1
LINKTYPE_RAW = 101


def read_pcapng(path):
    """Yields (linktype, packet bytes) for every packet in a pcapng file."""
    data = pathlib.Path(path).read_bytes()
    pos = 0
    endian = "<"
    linktypes = []
    while pos + 12 <= len(data):
        block_type = struct.unpack_from(endian + "I", data, pos)[0]
        if block_type == 0x0A0D0D0A:  # Section Header Block, defines byte order.
            magic = data[pos + 8:pos + 12]
            endian = "<" if magic == b"\x4d\x3c\x2b\x1a" else ">"
            linktypes = []
        block_len = struct.unpack_from(endian + "I", data, pos + 4)[0]
        body = data[pos + 8:pos + block_len - 4]
        if block_type == 1:  # Interface Description Block
            linktypes.append(struct.unpack_from(endian + "H", body, 0)[0])
        elif block_type == 6:  # Enhanced Packet Block
            iface, _, _, captured, _ = struct.unpack_from(endian + "IIIII", body, 0)
            yield linktypes[iface], body[20:20 + captured]
        elif block_type == 3:  # Simple Packet Block
            yield linktypes[0], body[4:]
        pos += block_len


def tcp_segments(path):
    """Yields (src, dst, seq, flags, payload) for every IPv4 TCP segment."""
    for linktype, pkt in read_pcapng(path):
        if linktype == LINKTYPE_ETHERNET:
            ethertype = struct.unpack_from(">H", pkt, 12)[0]
            offset = 14
            if ethertype == 0x8100:  # VLAN tag
                ethertype = struct.unpack_from(">H", pkt, 16)[0]
                offset = 18
            if ethertype != 0x0800:
                continue
            ip = pkt[offset:]
        elif linktype == LINKTYPE_RAW:
            ip = pkt
        else:
            continue
        if len(ip) < 20 or ip[0] >> 4 != 4 or ip[9] != 6:
            continue
        ihl = (ip[0] & 0x0F) * 4
        total_len = struct.unpack_from(">H", ip, 2)[0]
        src_ip = ".".join(str(b) for b in ip[12:16])
        dst_ip = ".".join(str(b) for b in ip[16:20])
        tcp = ip[ihl:total_len]
        src_port, dst_port, seq = struct.unpack_from(">HHI", tcp, 0)
        data_offset = (tcp[12] >> 4) * 4
        flags = tcp[13]
        yield (src_ip, src_port), (dst_ip, dst_port), seq, flags, tcp[data_offset:]


def reassemble(path, port):
    """Returns {(src, dst): bytes} for the TCP streams to and from the given port.

    Retransmitted and overlapping segments are dropped; only the first connection
    of every endpoint pair is kept.
    """
    streams = {}
    for src, dst, seq, flags, payload in tcp_segments(path):
        if port not in (src[1], dst[1]):
            continue
        key = (src, dst)
        stream = streams.setdefault(key, {"next": None, "data": bytearray()})
        if flags & 0x02:  # SYN consumes one sequence number.
            if stream["next"] is None:
                stream["next"] = (seq + 1) & 0xFFFFFFFF
            continue
        if not payload:
            continue
        if stream["next"] is None:
            stream["next"] = seq
        if seq != stream["next"]:
            continue
        stream["data"] += payload
        stream["next"] = (seq + len(payload)) & 0xFFFFFFFF
    return {key: bytes(s["data"]) for key, s in streams.items()}


def split_messages(stream):
    """Splits a stream into (id, payload) messages; a truncated tail is ignored."""
    messages = []
    pos = 0
    while pos + 3 <= len(stream):
        msg_id = stream[pos]
        length = struct.unpack_from("<H", stream, pos + 1)[0]
        if pos + 3 + length > len(stream):
            break
        messages.append((msg_id, stream[pos:pos + 3 + length]))
        pos += 3 + length
    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", help="pcapng file")
    parser.add_argument("--port", type=int, default=80, help="robot TCP port (default: 80)")
    parser.add_argument("--write", metavar="DIR", help="directory for the selected messages")
    parser.add_argument("--select", nargs="*", default=[], metavar="NAME:INDEX",
                        help="messages to write, e.g. iocontrol:0 (index among messages of that type)")
    args = parser.parse_args()

    by_name = {}
    for (src, dst), stream in reassemble(args.capture, args.port).items():
        direction = "pc->robot" if dst[1] == args.port else "robot->pc"
        messages = split_messages(stream)
        print(f"{src[0]}:{src[1]} -> {dst[0]}:{dst[1]} ({direction}): {len(stream)} bytes, {len(messages)} messages")
        counts = {}
        for msg_id, raw in messages:
            name = MESSAGE_NAMES.get(msg_id, f"id{msg_id}")
            counts[name] = counts.get(name, 0) + 1
            by_name.setdefault(name, []).append(raw)
        for name, n in counts.items():
            print(f"  {name}: {n}")

    if args.write:
        out_dir = pathlib.Path(args.write)
        out_dir.mkdir(parents=True, exist_ok=True)
        for item in args.select:
            name, _, index = item.partition(":")
            raw = by_name.get(name, [])[int(index or 0)]
            path = out_dir / f"{name}_{int(index or 0)}.bin"
            path.write_bytes(raw)
            print(f"wrote {path} ({len(raw)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
