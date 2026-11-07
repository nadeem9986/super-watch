# NWatch 6.7.2 — Back hold 400 ms

Derived from native NimBLE 6.7.1, which the user confirmed reconnects to iPhone
after powering off/on. Only firmware behavior change: GPIO20 long-press Back
threshold increased from 200 ms to 400 ms; still triggers while held, not upon
release. All native BLE, NVS bonds/settings, pairing OLED quiet guard, CPU80MHz,
-6 dBm controller power, modem sleep and 24FPS cap are retained.

Hardware: ESP32-C3 Super Mini; SH1106 128x64 OLED + DS3231 I2C SDA0/SCL1;
UP10, SELECT20, DOWN21; buttons use pullups and connect to GND.

## Flash update

App-only binary at **0x10000**, never 0x0. DIO, 40MHz, 4MB, no Erase All.
Keep the working 6.7/6.7.1 iPhone pairing; no Reset Pairing required.
For the public web installer use separate bootloader/partition/OTA/app files,
not a merged image with FF padding across NVS.

## Build

Open this folder in VS Code + PlatformIO. `platformio run` uses Arduino2.0.17
as a component of native ESP-IDF4.4.7, not NimBLE-Arduino. First build rebuilds
the framework and may need dependency downloads. sdkconfig enables GATT client
features through ROLE_CENTRAL, but the watch application does not scan or
initiate a physical BLE link; iPhone connects to the advertising watch.

The existing parser/quiet-policy tests are in tests/. Build passed; no hardware
upload or power measurement performed for 6.7.2. This does not guarantee source
notifications or all calls are delivered; real iPhone testing remains needed.

Original NWatch by Zak Kemble, ESP32-C3 rebuild for Huy Vector. GPLv3, see
License.txt. Preserve attribution/license when redistributing this source.

## ESP32 dev module port (SSD1306)

`src/main_esp32.cpp` is a separate build of the same watch for a classic ESP32
dev module with a 128x64 SSD1306 OLED, added after the C3 firmware above. The
original C3 source (`src/main.cpp`) is unchanged; `src/CMakeLists.txt` picks
the file that matches the build target, so each board keeps its own binary.

Differences from the C3 version:

- Display driver: SSD1306 (charge pump on, page mode, no SH1106 column offset)
- OLED + DS3231 I2C: SDA=GPIO21, SCL=GPIO22, OLED address 0x3C
- Buttons: LEFT=GPIO25, SELECT=GPIO26, RIGHT=GPIO27 (pullups to GND)
- BLE advertises as "NWatch 32"; same ANCS notification firmware otherwise

Build and flash with PlatformIO: `pio run -e esp32dev -t upload` (default env
`esp32-c3-supermini` still builds the C3 firmware). Compile-checked only;
display, buttons and iPhone pairing still need testing on real hardware.

