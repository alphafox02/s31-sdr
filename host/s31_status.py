#!/usr/bin/env python3
# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
"""Show what the board is doing right now: sample rate and format, receive
filter, LO and gain.

Sends a VITA 49.2 query-state command and decodes the reply. The reply
comes back to this script's own port, so it works while gqrx or another
SoapySDR application holds the stream's port.
"""
import argparse
import socket
import struct
import sys

STREAM_IDS = {4991: 0x53310001, 4993: 0x53310002}   # S31, RTL-SDR
CIF0_BW, CIF0_RF, CIF0_GAIN = 1 << 29, 1 << 27, 1 << 23
CIF0_RATE, CIF0_SE, CIF0_FMT = 1 << 21, 1 << 16, 1 << 15
CAM_REQ_QUERY = 1 << 18


def query(board, port):
    # one placeholder per requested field: bandwidth, RF, gain, rate,
    # State/Event, data format (two words)
    fields = struct.pack('>qqIqIII', 0, 0, 0, 0, 0, 0, 0)
    body = struct.pack('>IIII', STREAM_IDS.get(port, STREAM_IDS[4991]), CAM_REQ_QUERY, 1,
                       CIF0_BW | CIF0_RF | CIF0_GAIN | CIF0_RATE | CIF0_SE | CIF0_FMT) + fields
    packet = struct.pack('>I', (6 << 28) | (1 + len(body) // 4)) + body
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2)
    s.sendto(packet, (board, port))
    return s.recv(512)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--board', required=True, help="board's IPv4 address on the link")
    ap.add_argument('--port', type=int, default=4991)
    args = ap.parse_args()

    try:
        r = query(args.board, args.port)
    except socket.timeout:
        sys.exit(f'no reply from {args.board}:{args.port}')
    words = struct.unpack('>%dI' % (len(r) // 4), r)
    if len(words) < 15:
        sys.exit('unexpected reply: ' + ' '.join(f'{w:08x}' for w in words))

    def fix20(i):
        return struct.unpack('>q', r[4 * i:4 * i + 8])[0] / (1 << 20)

    # reply: header, stream ID, CAM, message ID, CIF0, then the fields
    bandwidth, rf, rate = fix20(5), fix20(7), fix20(10)
    gain = struct.unpack('>h', r[4 * 9 + 2:4 * 9 + 4])[0] / 128
    item_bits = (words[13] & 0x3F) + 1
    print(f'sample rate  {rate / 1e6:g} MS/s, {item_bits}-bit items')
    print(f'filter       {bandwidth / 1e6:g} MHz')
    print(f'LO           {rf / 1e6:.6f} MHz')
    print(f'gain         {"AGC" if gain < 0 else f"{gain:g}"}')


if __name__ == '__main__':
    main()
