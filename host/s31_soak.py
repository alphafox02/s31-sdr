#!/usr/bin/env python3
# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
"""Long-run soak logger for the ESP32-S31 VITA-49 stream.

Every minute, one line: data and context packets received, sample-count gaps
(true discontinuities, from the VRT sample counter), 4-bit packet-counter
breaks, kernel UDP drops, and the board's "status" reply (lock, monitor
repairs, DMA laps). Holds UDP 4991 while it runs.
"""
import argparse
import socket
import struct
import time


def udp_drops():
    with open('/proc/net/snmp') as f:
        rows = [line.split() for line in f if line.startswith('Udp:')]
    return int(rows[1][rows[0].index('RcvbufErrors')])


def board_status(ctl, board):
    try:
        ctl.sendto(b'status', (board, 4990))
        return ctl.recv(1024).decode()
    except socket.timeout:
        return 'no status reply'


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--board', required=True, help="board's IPv4 address on the link")
    ap.add_argument('--hours', type=float, default=12)
    ap.add_argument('--log', required=True, help='file to append to')
    args = ap.parse_args()

    data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 128 << 20)
    data.bind(('0.0.0.0', 4991))
    data.settimeout(2)
    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.settimeout(2)

    buf = bytearray(2048)
    end = time.time() + args.hours * 3600
    step = last_ts = last_cnt = None          # samples per data packet, learned from the stream
    with open(args.log, 'a', buffering=1) as log:
        log.write(f'# soak start {time.strftime("%Y-%m-%d %H:%M:%S")}, {args.hours} h\n')
        while time.time() < end:
            minute_end = time.time() + 60
            n_data = n_ctx = gaps = gap_samples = breaks = timeouts = 0
            drops0 = udp_drops()
            while time.time() < minute_end:
                try:
                    data.recv_into(buf)
                except socket.timeout:
                    timeouts += 1
                    continue
                ptype = buf[0] >> 4
                if ptype == 4:
                    n_ctx += 1
                    continue
                if ptype != 1:
                    continue
                n_data += 1
                cnt = buf[1] & 0xF
                ts = struct.unpack_from('>Q', buf, 8)[0]
                if step is None and last_ts is not None and ts > last_ts:
                    step = ts - last_ts
                if last_cnt is not None and cnt != (last_cnt + 1) & 0xF:
                    breaks += 1
                if step and last_ts is not None and ts != last_ts + step:
                    gaps += 1
                    gap_samples += ts - last_ts - step
                last_ts, last_cnt = ts, cnt
            log.write(f'{time.strftime("%H:%M:%S")} pkts {n_data} ctx {n_ctx} gaps {gaps} '
                      f'({gap_samples} samples) breaks {breaks} drops {udp_drops() - drops0} '
                      f'timeouts {timeouts} | {board_status(ctl, args.board)}\n')
        log.write(f'# soak end {time.strftime("%Y-%m-%d %H:%M:%S")}\n')


if __name__ == '__main__':
    main()
