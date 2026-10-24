# s31-sdr

**SDR firmware for the Espressif ESP32-S31**, streaming I/Q as VITA-49 over
Gigabit Ethernet.

This firmware turns Espressif's **ESP32-S31 Function CoreBoard** into a
receive-only SDR. The board captures I/Q from the ESP32-S31's own 2.4 GHz
radio and streams it as VITA-49 over its Gigabit Ethernet port. On your
computer, [SoapyVITA49](https://github.com/alphafox02/SoapyVITA49) picks
up the stream, so you can use the board from gqrx, GNU Radio, or anything
else that speaks SoapySDR.

There are two builds to choose from:

| Build | Samples | Sample rate | Filter | Network load |
| --- | --- | --- | --- | --- |
| 10-bit (default) | 10-bit I/Q, sent as CI16 | 20 MS/s | 20 MHz | ~680 Mbit/s |
| 8-bit | 8-bit I/Q, sent as CI8 | 40 MS/s | 40 MHz | ~680 Mbit/s |

Why not 10 bits at 40 MS/s? As CI16 that would take about 1.3 Gbit/s,
which is more than gigabit Ethernet can carry. So you get the full 10 bits
at 20 MS/s, or the full 40 MHz at 8 bits.

The stream is continuous. In an 8.5-hour test at 20 MS/s the host received
1.7 billion packets. There was one short gap of 5 packets (90 µs), and the
host itself dropped nothing.

You can tune anywhere from 2150 to 2850 MHz in steps of about 57 Hz, and
retuning doesn't interrupt the stream. The board's front end is designed for
2.4 GHz, though. Real signals have been received from 2160 to 2440 MHz, but
the top of the range hasn't been tested against a known signal yet.

Ten times a second the board sends a context packet with the sample rate,
frequency, bandwidth, gain and sample format, so there's nothing to set up
on the host side. You can change the frequency, gain (or AGC) and the
20/40 MHz filter through VITA 49.2 control, which SoapyVITA49 uses on its
own, or through a simple text port.
[docs/vita49.md](docs/vita49.md) covers both.

## What you need

- An ESP32-S31 Function CoreBoard.
- A computer with a gigabit Ethernet port, and a cable from the board
  straight to it. The commands here are for Linux.
- To use the stream in gqrx, GNU Radio or other SoapySDR applications:
  SoapySDR and [SoapyVITA49](https://github.com/alphafox02/SoapyVITA49).
  DragonOS already includes it (the `soapyvita49` package). On other
  systems, build it from source as its README describes. You need a
  version from October 2026 or later for the gain slider and AGC switch.
  Building and flashing don't need it.
- A USB-C cable to the port marked **UART**, for flashing and the console
  (115200 baud). The board's **DBG** port can power and flash it too, but
  the console only appears on UART. Opening the UART console resets the
  board, which is harmless: it comes straight back up and starts streaming
  again.

The firmware uses some free header pins internally to move samples around,
so **leave these pins unconnected**: GPIO 0-3, 21, 23, 25, 35-40 and 42-44,
plus 45-49 in the 10-bit build.

## Quick start

**1. Install ESP-IDF master** with the ESP32-S31 tools. You only do this
once, and it takes a while:

```sh
git clone --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
cd ~/esp/esp-idf && ./install.sh esp32s31
```

**2. Give your computer's Ethernet port an address** on the link to the
board, for example:

```sh
sudo ip addr add 192.168.50.1/24 dev <your-ethernet-interface>
```

That lasts until the interface goes down. To keep it, set the same manual
IPv4 address in your network settings. If you run a firewall, allow
incoming UDP on ports 4990 and 4991 from the board.

**3. Clone, build, flash and check the stream** with the board on USB and
Ethernet:

```sh
git clone https://github.com/alphafox02/s31-sdr.git
cd s31-sdr/firmware
. ~/esp/esp-idf/export.sh
python flash.py --board-ip 192.168.50.20 --host-ip 192.168.50.1
```

`flash.py` writes the addresses, builds the firmware, finds the board,
flashes it and then waits for the stream. When everything works it ends
with:

```
streaming: 20.00 MS/s (16-bit items, board reports 20 MS/s) at 2437.000 MHz
```

After that, `python flash.py` on its own rebuilds and reflashes with the
same addresses.

### About the addresses

The board has no DHCP; its address is built into the firmware.
`--board-ip` and `--host-ip` are saved to `firmware/main/local_net.h`,
which is gitignored, so your addresses never end up in the repository. In
that file `S31_GW` is your computer, not a router: it's where the stream
goes. To change addresses, run `flash.py` again with new ones.

When the board boots, it looks up your computer's MAC address and then
starts streaming to UDP port 4991. If your computer doesn't answer within
about 5 seconds, the console prints `no ARP reply from the host` and the
board gives up. Check the address on your computer and reset the board, or
just run `python flash.py` again.

### How flash.py picks the board

You don't have to work out which `/dev/tty*` the board is, or worry about
flashing something else by mistake. `flash.py` only opens ports whose USB
chip matches the ones on this board (Espressif's USB debug port and the
CP2102N UART bridge). Other ESP32 boards use those chips too, so it asks
each one what chip it is, which resets that device for a moment, and it
writes nothing unless the answer is ESP32-S31. With both cables plugged in
it flashes through the UART port. If it finds more than one S31 board, it
lists them and stops; run it again with `--mac` or `--port` to pick one.
`python flash.py --list` shows which board it would use without building
or flashing.

### Console

`flash.py` prints the board's UART port at the end. Run
`idf.py -p <port> monitor` to watch the console. Once the board is
streaming you'll see a line like this every second:

```
@@STREAM 55.6 kpkt/s  20.00 MS/s  lost 0 (total 0) ...
```

### 8-bit, 40 MS/s build

Change `STREAM_BITS` from 10 to 8 in
[firmware/main/main.c](firmware/main/main.c) and run `python flash.py`
again. If you edit `sdkconfig.defaults`, delete `sdkconfig` first so the
change takes effect.

## Receive

First, a little host setup. Linux's default UDP receive buffer is far too
small for these rates, so the kernel would drop packets. SoapyVITA49 asks
for a bigger buffer, but the kernel caps it unless you raise the limit:

```sh
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.netdev_max_backlog=250000
```

Then check that SoapySDR can see the board:

```sh
SoapySDRUtil --find="driver=vita49"
```

You should see a `VITA-49 ... sid 53310001` device at 20 Msps (40 Msps for
the 8-bit build). In gqrx, enter this as the device string:

```
driver=vita49,gain_range=0:127:1
```

gqrx then shows a gain slider that matches the board's 0-127 range, plus
the AGC switch, and tuning, gain and AGC go to the board over VITA 49.2.
Without `gain_range` the slider still works, but it spans SoapyVITA49's
default of about -256 to 256, of which the board only uses 0-127.

If you want to measure the stream directly (rate, gaps, dropped packets,
spectrum orientation, or a long logged run), the scripts in [host/](host/)
do that. Run them with your normal Python, not the one ESP-IDF sets up.

## Read before use

- **Receive only.** The firmware starts the ESP32-S31's Wi-Fi radio so it
  can receive, but it never joins a network or scans. Even so, don't cable
  the antenna port straight into sensitive test equipment without an
  attenuator.
- **Leave the listed header pins free** while it runs (see
  [What you need](#what-you-need)). The firmware drives them.
- **It depends on ESP-IDF master.** Capture goes through Espressif's RF test
  library, which ships with ESP-IDF as a binary and has no stable public
  API, so a future ESP-IDF update could break it. It was last tested on
  ESP-IDF master at commit `1c42e514` (2026-10-16).
- **No Espressif binaries live here.** The PHY, Wi-Fi and RF test libraries
  come from your own ESP-IDF install at build time.

## Limitations

- Two fixed sample rates: 20 MS/s (10-bit) and 40 MS/s (8-bit). Lower rates
  are left to the host, for example gqrx's own decimation.
- Gain is the receiver's raw gain index (0-127, higher is more gain), not
  calibrated in dB. Each step is roughly 0.35 dB between 20 and 50 and
  steeper above, and above about 80 the samples clip. Left to itself the
  AGC usually sits around 55-70. Values above 127 act as 127, and a
  negative value turns AGC on.
- Addresses are set at build time, and the board streams to one host.
- Tuning above about 2.5 GHz works but hasn't been checked against a known
  signal, and the board's front end is built for 2.4 GHz.

## What's where

- `firmware/` - the ESP-IDF application and `flash.py`.
- `host/` - Python tools for checking the stream.
- `tests/` - a host-side unit test for the DMA ring logic.
- `docs/` - the stream and control interface.

## Contributing

Issues and pull requests are welcome. When something doesn't work, include
the console's `@@STREAM` lines and the output of
`host/s31_vrt_check.py --seconds 10`; they say more than a description.
The DMA ring logic has a host-side test that runs without hardware:

```sh
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/main tests/test_dma_advance.c -o test_dma_advance
./test_dma_advance
```

## Related projects

- [SoapyVITA49](https://github.com/alphafox02/SoapyVITA49) - the SoapySDR
  driver that receives this stream.
- [openwrt-one-sdr](https://github.com/alphafox02/openwrt-one-sdr) - the
  same idea on the OpenWrt One router: 14-bit IQ at 40 MS/s over 2.5 GbE as
  VITA 49.

## License

Apache-2.0. Copyright (c) 2026 CEMAXECUTER LLC. See [LICENSE](LICENSE) and
[NOTICE](NOTICE). The firmware links Espressif's precompiled PHY, Wi-Fi and
RF test libraries, which come with ESP-IDF under Espressif's own license.
