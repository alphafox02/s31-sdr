# s31-sdr

**SDR firmware for the Espressif ESP32-S31**, streaming I/Q as VITA-49 over
Gigabit Ethernet.

Work in progress. This first version builds and boots on Espressif's
**ESP32-S31 Function CoreBoard** and brings up the 2.4 GHz receiver.

## Build

You'll need ESP-IDF **master** with the esp32s31 tools installed
(`./install.sh esp32s31` in your ESP-IDF folder). The S31 is still a preview
target, so `set-target` needs `--preview`:

```sh
. ~/esp/esp-idf/export.sh
cd firmware
idf.py --preview set-target esp32s31
idf.py build
idf.py -p <UART port> flash monitor
```

## License

Apache-2.0. Copyright (c) 2026 CEMAXECUTER LLC. See [LICENSE](LICENSE) and
[NOTICE](NOTICE). The firmware links Espressif's precompiled PHY, Wi-Fi and
RF test libraries, which come with ESP-IDF under Espressif's own license.
