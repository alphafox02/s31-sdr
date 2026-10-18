#!/usr/bin/env python3
# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
"""Long-run soak logger for the ESP32-S31 VITA-49 stream.

Every minute, one line: data and context packets received, sample-count gaps
(true discontinuities, from the VRT sample counter), 4-bit packet-counter
breaks, kernel UDP drops, and the board's "status" reply (lock, monitor
repairs, DMA laps). Holds UDP 4991 while it runs.
"""
import argparse, socket, struct, time

def udp_drops():
    rows = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')]
    return int(rows[1][rows[0].index('RcvbufErrors')])

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument('--board', required=True)
ap.add_argument('--hours', type=float, default=12)
ap.add_argument('--log', required=True)
a = ap.parse_args()
d = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
d.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 128 << 20); d.bind(('0.0.0.0', 4991)); d.settimeout(2)
c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); c.settimeout(2)
buf = bytearray(2048); end = time.time() + a.hours * 3600
step = None; last_ts = last_cnt = None
with open(a.log, 'a', buffering=1) as log:
    log.write(f'# soak start {time.strftime("%Y-%m-%d %H:%M:%S")}, {a.hours} h\n')
    while time.time() < end:
        t_end = time.time() + 60; data = ctx = gaps = breaks = timeouts = gap_samples = 0; dr0 = udp_drops()
        while time.time() < t_end:
            try: n = d.recv_into(buf)
            except socket.timeout: timeouts += 1; continue
            t = buf[0] >> 4
            if t == 4: ctx += 1; continue
            if t != 1: continue
            data += 1
            cnt = buf[1] & 0xF; ts = struct.unpack_from('>Q', buf, 8)[0]
            if step is None and last_ts is not None and ts > last_ts: step = ts - last_ts
            if last_cnt is not None and cnt != (last_cnt + 1) & 0xF: breaks += 1
            if step and last_ts is not None and ts != last_ts + step: gaps += 1; gap_samples += ts - last_ts - step
            last_ts, last_cnt = ts, cnt
        try:
            c.sendto(b'status', (a.board, 4990)); st = c.recv(1024).decode().split('-> ')[-1]
        except socket.timeout:
            st = 'no status reply'
        log.write(f'{time.strftime("%H:%M:%S")} pkts {data} ctx {ctx} gaps {gaps} ({gap_samples} samples) '
                  f'breaks {breaks} drops {udp_drops() - dr0} timeouts {timeouts} | {st}\n')
    log.write(f'# soak end {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
