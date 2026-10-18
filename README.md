# s31-sdr

**SDR firmware for the Espressif ESP32-S31**, streaming I/Q as VITA-49 over
Gigabit Ethernet.

Work in progress. On Espressif's **ESP32-S31 Function CoreBoard** it brings
up the 2.4 GHz receiver and captures 80 MS/s, 10-bit I/Q from the chip's
sample-dump engine. For now it prints each capture's signal power and AGC
gain index on the console once a second, and brings up Gigabit Ethernet
with a static address.

## Build

You'll need ESP-IDF **master** with the esp32s31 tools installed
(`./install.sh esp32s31` in your ESP-IDF folder). The S31 is still a preview
target, so `set-target` needs `--preview`:

```sh
. ~/esp/esp-idf/export.sh
cd firmware
cp main/local_net.h.example main/local_net.h    # then set the addresses
idf.py --preview set-target esp32s31
idf.py build
idf.py -p <UART port> flash monitor
```

## License

Apache-2.0. Copyright (c) 2026 CEMAXECUTER LLC. See [LICENSE](LICENSE) and
[NOTICE](NOTICE). The firmware links Espressif's precompiled PHY, Wi-Fi and
RF test libraries, which come with ESP-IDF under Espressif's own license.
