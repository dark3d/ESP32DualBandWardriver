# Dark3D Wardriver

Dual-band 2.4 and 5 GHz WiFi and BLE wardriving firmware for ESP32-C5 hardware. Logs to a
microSD card in WiGLE CSV format and uploads to WiGLE and WDG Wars.

Firmware is distributed here as binary releases. Source is not public.

## Download

Grab the latest `.bin` from [Releases](../../releases). Prereleases marked as such are tester
builds and are not served to devices over the air.

## Updating an existing device

Devices update themselves. On a unit that is already running Dark3D firmware, connect it to WiFi
and run the update check from the on-device menu. It pulls the current release from this repo and
reflashes itself.

## Flashing over USB

The release `.bin` is an application image. It updates a device that already has a bootloader and
partition table on it.

```
esptool --chip esp32c5 -p /dev/ttyUSB0 -b 460800 \
  write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0x10000 wardriver-c5-dark3d-<version>.bin \
  0x1f0000 wardriver-c5-dark3d-<version>.bin
```

Both offsets are written so that an over-the-air rollback cannot leave the device on a stale
image. Do not use `erase-flash`, which destroys saved settings including WiFi and upload
credentials.

## Reporting a problem

Open an issue. Useful details: firmware version from the device screen, what it was doing, and
the contents of `crash.log` and `debug.log` from the SD card if present.

## Licensing

The ESP32-C5 Wardriver firmware published here is built on
[ESP32DualBandWardriver](https://github.com/justcallmekoko/ESP32DualBandWardriver) by Just Call Me
Koko, used under the MIT license. See [NOTICE](NOTICE) for the required notices.
