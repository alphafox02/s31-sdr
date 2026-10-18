# Stream and control interface

The board sends to the host at `S31_GW` (see `firmware/main/local_net.h`).
Everything is UDP on the direct Ethernet link:

| Port | Direction | Content |
| --- | --- | --- |
| 4991 | board -> host | VRT data and context packets |
| 4991 | host <-> board | VITA 49.2 command / acknowledgement packets |
| 4990 | host <-> board | plain-text control |

At boot the board also sends a few 3-byte datagrams to the host's port
4992. They exist only to make the host answer ARP, so the board learns its
MAC address, and can be ignored.

Field encodings follow SoapyVITA49 (VRT.cpp, Control.cpp), so the stream is
discovered and decoded with no settings.

## Data packets

- Packet type 1 (IF data with stream ID), stream ID `0x53310001`.
- 4-bit packet counter, used for loss detection.
- Timestamp: TSI none, TSF = sample count (64-bit), so a gap in the stream
  shows up as a jump in the count.
- Payload: 1440 bytes. That is 720 CI8 pairs (8-bit build) or 360 CI16 pairs
  (10-bit build). Each frame is 1498 bytes, about 55,600 frames/s in both
  builds.
- 10-bit samples are sent as CI16 with the 10 data bits at the top of each
  16-bit item (low 6 bits zero), so they scale like ordinary 16-bit samples.
- The spectrum is normally oriented (absolute frequency = RF reference +
  baseband offset).
- The sample rate equals the filter width and there is no decimation
  filter after the receive filter, so the filter's edges (about +-11 MHz
  with the 20 MHz filter) fold in slightly at the edges of the band.

## Context packets

Packet type 4, the same stream ID, sent every 100 ms and immediately after any
change. CIF0 fields:

| Field | Value |
| --- | --- |
| Bandwidth | receive filter, 20 or 40 MHz |
| RF reference frequency | the LO |
| Gain | stage 1 = forced AGC gain index; -1 = AGC on |
| Sample rate | 20 MS/s (10-bit) or 40 MS/s (8-bit) |
| State/Event | AGC / manual gain indicator |
| Data payload format | CI16 or CI8, complex, signed fixed point |

## VITA 49.2 control

The board answers command packets (types 6/7) on port 4991, replying from the
same port. A query-state command is answered with whichever of these fields
it asks for: bandwidth, RF reference, gain, sample rate, State/Event and
data format. Every other command is applied (there is no validate-only
mode) and acknowledged when it asks for an acknowledgement:

| Field | Effect |
| --- | --- |
| RF reference | tune exactly, 2150-2850 MHz; outside that range the command is refused and the receiver is left unchanged |
| Bandwidth | receive filter: values above 20 MHz select 40 MHz, others 20 MHz (see below) |
| Gain | stage 1 >= 0 forces that gain index (0-127); negative turns AGC on |
| State/Event AGC bit | AGC on / off (off restores the last manual index) |
| Sample rate | fixed per build; any other value is acknowledged as partial |

If the result differs from the request, the acknowledgement is marked
partial, and errors are flagged. Retuning does not interrupt the stream.

Each build starts with the filter that matches its sample rate, and either
filter can be chosen in either build. Choosing 40 MHz in the 10-bit,
20 MS/s build lets signals 10-20 MHz from the centre alias into the band.
In the 8-bit build, a host that asks for 20 MHz or less switches it to the
20 MHz filter.

## Text control (port 4990)

Send one command per datagram; the board replies with one line.

| Command | Effect |
| --- | --- |
| `f <Hz>` | tune exactly, 2150-2850 MHz |
| `ch <1..13>` | tune to a 2.4 GHz Wi-Fi channel centre (`err` if the tune fails) |
| `bw <20\|40>` | receive filter width in MHz |
| `gain <0..127>` | force the AGC gain index |
| `gain auto` | AGC on |
| `agc` | gain index range over a short capture |
| `status` | LO, PLL lock, gain, and radio monitor counters |

Anything else replies `err`. The firmware also has low-level debugging
commands (raw register access, capture test hooks). They have no
authentication, so they are compiled out unless you build with
`S31_DEBUG_CMDS` set to 1 in `firmware/main/stream.c`.

A radio monitor runs every 500 ms. If anything disturbs the synthesizer,
the VCO lock or a forced gain, it puts them back, and `status` reports how
often it had to.
