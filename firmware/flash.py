#!/usr/bin/env python3
# Copyright 2026 CEMAXECUTER LLC
# SPDX-License-Identifier: Apache-2.0
"""Build the firmware, flash it onto an ESP32-S31 board (and only onto one),
and check that the stream arrives.

Run it from the ESP-IDF environment (it uses that environment's idf.py and
esptool). The first time, give the board's address and your computer's:

    python flash.py --board-ip 192.168.50.20 --host-ip 192.168.50.1

That writes main/local_net.h, builds, flashes and then waits for the board's
VITA-49 stream. Later runs reuse local_net.h, so `python flash.py` is enough.
`python flash.py --list` only shows which board would be flashed.

Only ports with the USB IDs this board uses are opened: Espressif's USB
Serial/JTAG (303a:1001, the DBG port) and the Silicon Labs CP2102N
(10c4:ea60, the UART port). Other ESP32 boards can use the same chips, so
each such port is asked for its chip type, which resets that device for a
moment (also with --list), and anything that is not an ESP32-S31 is left
alone. The DBG port's USB serial number is the chip's MAC, so a board's DBG
and UART ports are recognised as one board, flashed through UART when both
are there. If more than one S31 board is connected, nothing is flashed
until you pick one with --mac or --port.
"""
import argparse
import ipaddress
import json
import os
import re
import socket
import struct
import subprocess
import sys
import time

try:
    import serial.tools.list_ports
    import esptool
    from esptool.cmds import detect_chip
except ImportError:
    sys.exit('esptool / pyserial not found: run ESP-IDF\'s export.sh first')

HERE = os.path.dirname(os.path.abspath(__file__))
NET_H = os.path.join(HERE, 'main', 'local_net.h')
USB_JTAG = (0x303A, 0x1001)
CP2102N = (0x10C4, 0xEA60)
CHIP = 'ESP32-S31'
VRT_PORT = 4991
STREAM_ID = 0x53310001


def candidates(only=None):
    only = os.path.realpath(only) if only else None       # /dev/serial/by-id/... links
    ports = []
    for p in serial.tools.list_ports.comports():
        if only and os.path.realpath(p.device) != only:
            continue
        if (p.vid, p.pid) in (USB_JTAG, CP2102N):
            ports.append(p)
    return ports


def probe(port):
    """Chip name and base MAC of the device on `port`; it is reset back into
    its firmware afterwards. (None, None) if it does not answer."""
    try:
        esp = detect_chip(port.device, connect_attempts=3)
    except Exception:
        return None, None
    try:
        mac = esp.read_mac('BASE_MAC')
        mac = ':'.join(f'{b:02X}' for b in mac) if mac else None
        return esp.CHIP_NAME, mac
    except Exception:
        return esp.CHIP_NAME, None
    finally:
        try:
            esp.hard_reset()
        except Exception:
            pass
        esp._port.close()


def read_net():
    """(board, netmask, host) from local_net.h, or None."""
    try:
        text = open(NET_H).read()
    except OSError:
        return None
    v = dict(re.findall(r'#define\s+(S31_IP|S31_NETMASK|S31_GW)\s+"([^"]*)"', text))
    if len(v) != 3:
        sys.exit(f'{NET_H} does not define S31_IP, S31_NETMASK and S31_GW')
    return v['S31_IP'], v['S31_NETMASK'], v['S31_GW']


def write_net(board, netmask, host):
    try:
        b, h = ipaddress.IPv4Address(board), ipaddress.IPv4Address(host)
        net = ipaddress.IPv4Network(f'{host}/{netmask}', strict=False)
    except ValueError as e:
        sys.exit(f'bad address: {e}')
    if b == h or b not in net:
        sys.exit(f'the board ({board}) and your computer ({host}) need different addresses '
                 f'on the same subnet ({net})')
    with open(NET_H, 'w') as f:
        f.write('/* Written by flash.py. The board\'s address, and the host that receives\n'
                ' * the stream (S31_GW). Gitignored. */\n'
                '#pragma once\n'
                f'#define S31_IP       "{board}"\n'
                f'#define S31_NETMASK  "{netmask}"\n'
                f'#define S31_GW       "{host}"\n')


def host_has_address(host):
    """True/False if this machine has `host` on an interface; None if unknown."""
    try:
        out = subprocess.run(['ip', '-o', '-4', 'addr'], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return any(f' {host}/' in line for line in out.splitlines())


def idf(*args):
    idf_py = os.path.join(os.environ.get('IDF_PATH', ''), 'tools', 'idf.py')
    if not os.path.isfile(idf_py):
        sys.exit('idf.py not found: run ESP-IDF\'s export.sh first')
    r = subprocess.call([sys.executable, idf_py, *args], cwd=HERE)
    if r:
        sys.exit(f'idf.py {" ".join(args)} failed')


def build():
    try:
        target = re.search(r'^CONFIG_IDF_TARGET="([^"]*)"', open(os.path.join(HERE, 'sdkconfig')).read(), re.M)
    except OSError:
        target = None
    if not target or target.group(1) != 'esp32s31':
        idf('--preview', 'set-target', 'esp32s31')
    idf('build')


def check_stream(board, seconds):
    """Wait for the board's context packet, then count data for 2 s."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('0.0.0.0', VRT_PORT))
    except OSError:
        print(f'stream check skipped: UDP {VRT_PORT} is in use (close SoapySDR applications)')
        return True
    s.settimeout(0.5)
    print(f'waiting up to {seconds} s for the stream from {board} ...', flush=True)
    end = time.time() + seconds
    ctx = None
    while time.time() < end and ctx is None:
        try:
            p, src = s.recvfrom(2048)
        except socket.timeout:
            continue
        if src[0] != board or len(p) < 60 or struct.unpack('>I', p[4:8])[0] != STREAM_ID:
            continue
        if p[0] >> 4 == 4:                                  # context packet
            fix = lambda o: struct.unpack('>q', p[o:o + 8])[0] / (1 << 20)
            ctx = dict(rf=fix(28), rate=fix(40), item=(struct.unpack('>I', p[52:56])[0] & 0x3F) + 1)
    if ctx is None:
        print(f'no stream from {board} within {seconds} s. Check that your computer\'s Ethernet\n'
              f'port has the S31_GW address, that the cable is in, and that no firewall\n'
              f'blocks UDP {VRT_PORT}; the console (idf.py -p <UART port> monitor) says more.')
        return False
    n, t0 = 0, time.time()
    while time.time() - t0 < 2:
        try:
            p, src = s.recvfrom(2048)
        except socket.timeout:
            continue
        if src[0] == board and p[0] >> 4 == 1:
            n += 1
    rate = n * 1440 / (2 * ctx['item'] // 8) / 2 / 1e6
    print(f'streaming: {rate:.2f} MS/s ({ctx["item"]}-bit items, board reports '
          f'{ctx["rate"] / 1e6:.0f} MS/s) at {ctx["rf"] / 1e6:.3f} MHz')
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].replace('\n', ' '))
    ap.add_argument('--board-ip', help="the board's address (writes main/local_net.h)")
    ap.add_argument('--host-ip', help='your computer\'s address on the link: the stream goes here')
    ap.add_argument('--netmask', default='255.255.255.0')
    ap.add_argument('--port', help='use only this serial port (still checked)')
    ap.add_argument('--mac', help='pick the board with this MAC when several are connected')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--no-build', action='store_true', help='flash the existing build as it is')
    ap.add_argument('--no-check', action='store_true', help="don't wait for the stream after flashing")
    ap.add_argument('--list', action='store_true', help='show the board that would be flashed and exit')
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)     # keep our lines in order with idf.py's

    if not args.list:
        if args.board_ip or args.host_ip:
            if not (args.board_ip and args.host_ip):
                sys.exit('give both --board-ip and --host-ip')
            write_net(args.board_ip, args.netmask, args.host_ip)
        net = read_net()
        if net is None:
            sys.exit('no network addresses yet. Run once with, for example:\n'
                     '    python flash.py --board-ip 192.168.50.20 --host-ip 192.168.50.1')
        board_ip, _, host_ip = net
        print(f'board {board_ip}, stream to {host_ip}')
        if host_has_address(host_ip) is False:
            print(f'note: no interface on this computer has {host_ip}; give your Ethernet port that '
                  f'address\n      (e.g. sudo ip addr add {host_ip}/24 dev <interface>) or the '
                  f'stream will not arrive')

    # Look for the board's USB chips before a long build (nothing is opened yet).
    if not candidates(args.port):
        sys.exit(f'no ESP32-S31 port found (looked for USB {USB_JTAG[0]:04x}:{USB_JTAG[1]:04x} '
                 f'and {CP2102N[0]:04x}:{CP2102N[1]:04x}' + (f' on {args.port}' if args.port else '') + ')')

    if not args.list:
        if not args.no_build:
            build()
        bdir = os.path.join(HERE, 'build')
        try:
            with open(os.path.join(bdir, 'flasher_args.json')) as f:
                fa = json.load(f)
            extra = fa['extra_esptool_args']
            files = fa['flash_files']
            if extra['chip'] != 'esp32s31':
                sys.exit(f'the build in {bdir} is for {extra["chip"]}, not esp32s31')
        except OSError:
            sys.exit(f'no build in {bdir}: run without --no-build')
        except (ValueError, KeyError, TypeError):
            sys.exit(f'{bdir}/flasher_args.json is not a usable ESP-IDF build: run without --no-build')

    ports = candidates(args.port)
    try:
        esptool.log.set_verbosity('silent')
    except Exception:
        pass
    boards = {}
    # UART bridges first. A DBG port's USB serial number is its chip's MAC,
    # so one that matches a board already found needs no probe (probing it
    # would only reset that board again, and the DBG port drops out while
    # the chip resets).
    ports.sort(key=lambda p: (p.vid, p.pid) == USB_JTAG)
    for p in ports:
        kind = 'DBG' if (p.vid, p.pid) == USB_JTAG else 'UART'
        if kind == 'DBG' and p.serial_number and p.serial_number.upper() in boards:
            print(f'  {p.device:14s} {kind:4s}  same board, MAC {p.serial_number.upper()}')
            boards[p.serial_number.upper()][kind] = p.device
            continue
        name, mac = probe(p)
        if name != CHIP:
            print(f'  {p.device:14s} {kind:4s}  {name or "no answer"} - not touched further')
            continue
        print(f'  {p.device:14s} {kind:4s}  {name}  MAC {mac or "unreadable"}')
        # a board whose MAC could not be read counts as a board of its own
        boards.setdefault(mac or f'unknown MAC on {p.device}', {})[kind] = p.device

    if args.mac:
        boards = {m: b for m, b in boards.items() if m and m.upper() == args.mac.upper()}
    if not boards:
        sys.exit(f'no {CHIP} board found' + (f' with MAC {args.mac}' if args.mac else ''))
    if len(boards) > 1:
        sys.exit(f'{len(boards)} {CHIP} boards found, so nothing was flashed. '
                 'Pick one with --mac <MAC> or --port <port>.')
    mac, ports = next(iter(boards.items()))
    port = ports.get('UART') or ports['DBG']      # the UART bridge flashes more reliably
    if args.list:
        print(f'would flash board {mac} through {port}')
        return

    cmd = [sys.executable, '-m', 'esptool', '--chip', 'esp32s31', '-p', port, '-b', str(args.baud),
           '--before', extra['before'], '--after', extra['after']]
    if not extra.get('stub', True):
        cmd.append('--no-stub')
    cmd += ['write-flash'] + fa['write_flash_args']
    for addr, path in sorted(files.items(), key=lambda kv: int(kv[0], 16)):
        cmd += [addr, path]
    print(f'flashing board {mac} through {port}', flush=True)
    if subprocess.call(cmd, cwd=bdir):
        sys.exit('flashing failed')
    print(f'flashed. Console: idf.py -p {port} monitor')
    if not args.no_check and not check_stream(board_ip, 30):
        sys.exit(1)


if __name__ == '__main__':
    main()
