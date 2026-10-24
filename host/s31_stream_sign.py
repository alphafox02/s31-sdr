#!/usr/bin/env python3
# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
"""Check the spectral orientation of the live VITA-49 stream across LOs.

Retunes the board over its control port ("ch N"), and for each channel keeps
1 s of data that arrives after a context packet reports the new RF
reference. External signals sit at fixed absolute frequencies, so mapping each
LO's max-hold spectrum with the right sign (absolute = LO + offset for a
correctly oriented stream) makes the overlaps agree. The sample format and
rate come from the board's context packets, and the board is tuned back to
its starting frequency at the end.
"""
import argparse
import socket
import struct
import time
from itertools import combinations

import numpy as np

NFFT = 1024
HALF_SPAN = 9e6


def fix20(b):
    return struct.unpack('>q', b)[0] / float(1 << 20)


def rf_ref_of(ctx):
    w0 = struct.unpack('>I', ctx[:4])[0]
    i = 8 + (4 if (w0 >> 22) & 3 else 0) + (8 if (w0 >> 20) & 3 else 0)
    cif0 = struct.unpack('>I', ctx[i:i + 4])[0]; i += 4
    if cif0 & (1 << 29): i += 8
    if cif0 & (1 << 28): i += 8
    return fix20(ctx[i:i + 8]) if cif0 & (1 << 27) else None


def stream_format(data_sock):
    """(RF reference, sample rate, item bits) from the next context packet,
    in the layout this firmware sends (CIF0: bandwidth, RF, gain, rate,
    State/Event, format)."""
    data_sock.settimeout(2.0)
    while True:
        p = data_sock.recv(2048)
        if p[0] >> 4 == 4 and len(p) >= 60:
            return fix20(p[28:36]), fix20(p[40:48]), (struct.unpack('>I', p[52:56])[0] & 0x3F) + 1


def capture(data_sock, want_lo, seconds, rate, bits):
    data_sock.settimeout(2.0)
    t_end = None
    chunks, n = [], 0
    while True:
        p = data_sock.recv(2048)
        ptype = p[0] >> 4
        if ptype == 4 and t_end is None:
            lo = rf_ref_of(p)
            if lo is not None and abs(lo - want_lo) < 1e3:      # the exact tune lands within the 57 Hz synthesizer step
                t_end = time.time() + seconds
            continue
        if ptype == 1 and t_end is not None:
            chunks.append(p[16:]); n += (len(p) - 16) // (bits // 4)
            if time.time() >= t_end or n >= seconds * rate:
                break
    items = np.frombuffer(b''.join(chunks), dtype='>i2' if bits == 16 else np.int8)
    if bits == 16:
        check_items(items)
    raw = items.astype(np.float32)
    x = raw[0::2] + 1j * raw[1::2]
    win = np.hanning(NFFT)
    p = np.zeros(NFFT)
    for k in range(min(len(x) // NFFT, 20000)):
        s = x[k * NFFT:(k + 1) * NFFT]
        p = np.maximum(p, np.abs(np.fft.fft((s - s.mean()) * win)) ** 2)
    return 10 * np.log10(np.fft.fftshift(p) + 1e-9)


def check_items(items):
    """10-bit items in 16: the low 6 bits must be zero and the two lowest
    data bits (6, 7) must both toggle, for I and for Q."""
    u = items.view('>u2')
    junk = int(np.count_nonzero(u & 0x3F))
    for name, v in (('Q', u[0::2]), ('I', u[1::2])):
        b6, b7 = ((v >> 6) & 1).mean(), ((v >> 7) & 1).mean()
        print(f'    {name}: bit 6 set {b6:.3f}, bit 7 set {b7:.3f}', end='')
    print(f'; items with low 6 bits nonzero: {junk}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--board', required=True, help="board's IPv4 address on the link")
    ap.add_argument('--port', type=int, default=4991)
    # 5 MHz steps: at 20 MS/s, 10 MHz steps leave too little overlap for a
    # reliable answer
    ap.add_argument('--channels', default='3,4,5,6,7,8,9')
    args = ap.parse_args()

    data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024 * 1024)
    data.bind(('0.0.0.0', args.port))
    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.settimeout(2.0)
    start_lo, rate, bits = stream_format(data)
    print(f'stream: {bits}-bit items, {rate / 1e6:.0f} MS/s, at {start_lo / 1e6:.3f} MHz')

    spectra = {}
    try:
        for ch in [int(c) for c in args.channels.split(',')]:
            ctl.sendto(f'ch {ch}'.encode(), (args.board, args.port - 1))
            print(f'ch{ch}: board replied {ctl.recv(64).decode()}')
            lo = (2407 + 5 * ch) * 1e6
            spectra[lo] = capture(data, lo, 1.0, rate, bits)
    finally:
        ctl.sendto(f'f {start_lo:.0f}'.encode(), (args.board, args.port - 1))
        print(f'tuned back to {start_lo / 1e6:.3f} MHz: {ctl.recv(128).decode()}')

    off = np.fft.fftshift(np.fft.fftfreq(NFFT, 1 / rate))
    keep = np.abs(off) <= HALF_SPAN
    los = sorted(spectra)
    grid = np.arange(los[0] - HALF_SPAN, los[-1] + HALF_SPAN, 50e3)
    scores = {}
    for sign in (+1, -1):
        curves = {}
        for lo, db in spectra.items():
            f = lo + sign * off[keep]
            o = np.argsort(f)
            curves[lo] = np.interp(grid, f[o], db[keep][o], left=np.nan, right=np.nan)
        rs = []
        for a, b in combinations(curves, 2):
            m = ~np.isnan(curves[a]) & ~np.isnan(curves[b])
            if m.sum() > 20:
                rs.append(np.corrcoef(curves[a][m], curves[b][m])[0, 1])
        scores[sign] = float(np.mean(rs))
        print(f'sign {sign:+d}: mean overlap correlation {scores[sign]:+.3f} ({len(rs)} LO pairs)')
    best = max(scores, key=scores.get)
    print('=> stream orientation', 'CORRECT (absolute = LO + offset)' if best > 0 else
          'MIRRORED (absolute = LO - offset)', f'(margin {abs(scores[1] - scores[-1]):.3f})')


if __name__ == '__main__':
    main()
