# firmware/

ESP-IDF application for the ESP32-S31 (RISC-V). Build and flash
instructions are in the [top-level README](../README.md#quick-start).

- `main.c` - brings up the PHY and Ethernet, then starts the streamer.
- `stream.c` - capture, VRT packetizer, VITA 49.2 and text control, tuning,
  gain, filter, radio monitor, and the send queue a second stream uses.
- `vrt.h` - VITA 49 helpers shared by both streams.
- `usb_rtl.c` - USB host and the RTL-SDR stream on port 4993.
- `rtl_drv.c` - the RTL2832U and R820T/R820T2 driver.
- `Kconfig.projbuild` - the option that leaves the RTL-SDR support out.
- `cam_lsb.c` - camera-input capture of the 10-bit build's low bits.
- `dma_pos.c` - exact DMA ring write positions.
- `net.c` - Ethernet (EMAC + YT8531) with a static address.
- `dump_bank.c` - reserves the receiver's sample bank in SRAM.
- `local_net.h.example` - copy to `local_net.h` and set the addresses.
- `flash.py` - sets the addresses, builds, finds the board (and only the
  board), flashes it and checks the stream.
