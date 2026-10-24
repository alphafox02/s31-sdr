#!/usr/bin/env python3
# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
"""Check the ESP32-S31 VITA-49 stream: rate, continuity, context, spectrum.

Listens on a UDP port (default 4991) for the firmware's VRT packets:
  data    type 1, stream ID, TSF sample count, CI8 or CI16 payload
  context type 4, CIF0 = bandwidth, RF reference, sample rate, data format
Reports packet and sample rates, 4-bit packet-counter breaks, sample-count
gaps, the decoded context, kernel UDP drops, and optionally writes a spectrum
of the received samples (--png).
"""
import argparse
import socket
import struct
import time

import numpy as np


def fix20(hi_lo):
    return struct.unpack('>q', hi_lo)[0] / float(1 << 20)


def udp_drops():
    with open('/proc/net/snmp') as f:
        rows = [l.split() for l in f if l.startswith('Udp:')]
    h, v = rows[0], rows[1]
    return int(v[h.index('RcvbufErrors')])


def parse_context(p):
    w0 = struct.unpack('>I', p[:4])[0]
    words = w0 & 0xFFFF
    i = 4 + 4                                     # stream ID
    tsi, tsf = (w0 >> 22) & 3, (w0 >> 20) & 3
    i += 4 if tsi else 0
    i += 8 if tsf else 0
    cif0 = struct.unpack('>I', p[i:i + 4])[0]; i += 4
    out = {}
    if cif0 & (1 << 29): out['bandwidth'] = fix20(p[i:i + 8]); i += 8
    if cif0 & (1 << 28): out['if_ref'] = fix20(p[i:i + 8]); i += 8
    if cif0 & (1 << 27): out['rf_ref'] = fix20(p[i:i + 8]); i += 8
    if cif0 & (1 << 26): i += 8                                  # RF offset
    if cif0 & (1 << 25): i += 8                                  # IF offset
    if cif0 & (1 << 24): i += 4                                  # reference level
    if cif0 & (1 << 23):                                         # gain, stage 1
        out['gain'] = struct.unpack('>h', p[i + 2:i + 4])[0] / 128; i += 4
    if cif0 & (1 << 22): i += 4                                  # over-range count
    if cif0 & (1 << 21): out['sample_rate'] = fix20(p[i:i + 8]); i += 8
    if cif0 & (1 << 20): i += 8                                  # timestamp adjustment
    if cif0 & (1 << 19): i += 4                                  # timestamp calibration time
    if cif0 & (1 << 18): i += 4                                  # temperature
    if cif0 & (1 << 17): i += 8                                  # device identifier
    if cif0 & (1 << 16):                                         # State/Event
        se = struct.unpack('>I', p[i:i + 4])[0]; i += 4
        if (se >> 28) & 1: out['agc'] = 'on' if (se >> 16) & 1 else 'off'
    if cif0 & (1 << 15):
        w1 = struct.unpack('>I', p[i:i + 4])[0]
        out['format'] = (f"{'complex' if (w1 >> 29) & 3 == 1 else 'real?'} "
                         f"item {(w1 & 0x3F) + 1} bits / field {((w1 >> 6) & 0x3F) + 1} bits")
        out['item_bits'] = (w1 & 0x3F) + 1
    return out, words * 4 == len(p)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', type=int, default=4991)
    ap.add_argument('--seconds', type=float, default=8)
    ap.add_argument('--png', help='write a spectrum of the received samples here')
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024 * 1024)
    s.bind(('0.0.0.0', args.port))
    s.settimeout(30.0)                # waiting for the stream to start
    buf = bytearray(2048)
    view = memoryview(buf)

    drops0 = udp_drops()
    n_data = n_ctx = cnt_breaks = ts_gaps = n_early = 0
    last_cnt = last_ts = None
    ctx = None
    keep = []                     # up to ~1 s of samples for the spectrum
    keep_n, keep_max = 0, 40_000_000
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        try:
            n = s.recv_into(buf)
        except socket.timeout:
            print('no packets (timeout)'); break
        if n_data == 0 and n_ctx == 0:
            s.settimeout(2.0); t0 = time.time()
        w0 = struct.unpack_from('>I', buf, 0)[0]
        ptype, cnt = w0 >> 28, (w0 >> 16) & 0xF
        if ptype == 4:
            n_ctx += 1
            if ctx is None:
                ctx, size_ok = parse_context(bytes(view[:n]))
                ctx['size_field_ok'] = size_ok
            continue
        if ptype != 1:
            continue
        n_data += 1
        ts = struct.unpack_from('>Q', buf, 8)[0]
        if last_cnt is not None and cnt != (last_cnt + 1) & 0xF:
            cnt_breaks += 1
        if ctx is None:               # item size unknown until the first context packet
            last_cnt = cnt
            n_early += 1
            continue
        npairs = (n - 16) // (2 * (ctx['item_bits'] // 8))
        if last_ts is not None and ts != last_ts + npairs:
            ts_gaps += 1
        last_cnt, last_ts = cnt, ts
        if args.png and keep_n < keep_max:
            keep.append(bytes(view[16:n])); keep_n += npairs
    dt = time.time() - t0
    drops = udp_drops() - drops0

    pairs_per_pkt = 1440 // (2 * ((ctx or {}).get('item_bits', 8) // 8))
    print(f'{n_data} data + {n_ctx} context packets in {dt:.1f} s: '
          f'{n_data / dt / 1e3:.1f} kpkt/s = {n_data * pairs_per_pkt / dt / 1e6:.2f} MS/s received')
    print(f'packet-counter breaks: {cnt_breaks}   sample-count gaps: {ts_gaps} '
          f'(checked after the first context; {n_early} packets before it)   '
          f'kernel UDP receive drops: {drops}')
    if ctx:
        print('context:', ', '.join(f'{k} {v / 1e6:.3f} MHz' if isinstance(v, float) and k != 'gain' else f'{k} {v}'
                                    for k, v in ctx.items()))

    if args.png and keep and ctx:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        dt_item = '>i2' if ctx.get('item_bits', 8) == 16 else np.int8
        raw = np.frombuffer(b''.join(keep), dtype=dt_item).astype(np.float32)
        x = raw[0::2] + 1j * raw[1::2]
        nfft = 2048
        segs = len(x) // nfft
        win = np.hanning(nfft)
        p = np.zeros(nfft)
        for k in range(min(segs, 4000)):
            seg = x[k * nfft:(k + 1) * nfft]
            p = np.maximum(p, np.abs(np.fft.fft((seg - seg.mean()) * win)) ** 2)
        f = (np.fft.fftshift(np.fft.fftfreq(nfft, 1 / ctx['sample_rate'])) + ctx['rf_ref']) / 1e6
        plt.figure(figsize=(12, 5))
        plt.plot(f, 10 * np.log10(np.fft.fftshift(p) + 1e-9))
        plt.xlabel('frequency (MHz)')
        plt.ylabel('max-hold dB')
        plt.title(f'ESP32-S31 VITA-49 stream, {ctx.get("item_bits", 8)}-bit items, {ctx["sample_rate"] / 1e6:.0f} MS/s, '
                  f'{len(x) / ctx["sample_rate"]:.2f} s received')
        plt.grid(True, alpha=0.3)
        plt.savefig(args.png, dpi=110, bbox_inches='tight')
        print('wrote', args.png)


if __name__ == '__main__':
    main()
