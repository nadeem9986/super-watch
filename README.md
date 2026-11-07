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
