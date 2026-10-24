# host/

Python 3 tools for checking the stream. Run them with your system Python,
not ESP-IDF's. The first two need numpy, and `--png` also needs matplotlib.

- `s31_vrt_check.py [--port 4991] [--seconds 8] [--png out.png]` - packet and
  sample rates, packet-counter breaks, sample-count gaps, decoded context,
  kernel UDP drops, and an optional spectrum. Works with either build.
- `s31_stream_sign.py --board <ip>` - retunes across a few channels and
  checks that the spectrum is the right way round (and, in the 10-bit
  build, the sample layout), then tunes the board back to where it was.
  It reads the format and rate from the stream, so it works with either
  build.
- `s31_soak.py --board <ip> --log <file> [--hours 12]` - one line a minute
  for long runs: packets, gaps, drops and the board's `status`.

Each tool listens on UDP 4991, so close any SoapySDR application first.
