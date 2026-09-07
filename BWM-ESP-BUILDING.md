# Building & Flashing the BWM (ESP32-C2) Firmware

The Blue/Wireless Module (BWM) on the Proxmark5 runs an **ESP32-C2**. Its
firmware lives in the `Proxmark5_BWM_esp32` repo (separate from the main
proxmark3 tree) and is built with **Espressif's ESP-IDF**, not the ARM
toolchain used for the PM5 itself.

There are two ways to get firmware onto it:

- **OTA over the PM5 link** — `hw bwm upgrade` from the pm3 client. No wires.
  Works on a BWM that still boots and responds. **This is the normal path.**
- **Serial (esptool) via the 5-pin header** — for first provisioning or
  recovering a bricked module. Needs physical access to the header.

You only need ESP-IDF (this guide) if you want to **build** the firmware
yourself. To just flash a prebuilt `.bin`, jump to [Flashing](#flashing).

---

## 1. Prerequisites — the IDF version matters

The firmware targets **ESP-IDF v5.5.x** (built/tested against **v5.5.2**).

> ⚠️ Use v5.5.x. On **IDF < 5.3** the build fails with
> `The component esp_driver_uart could not be found` — that component was
> split out of `driver` in 5.3, so an older IDF simply doesn't have it.
> This is the single most common build failure; check your IDF version first:
>
> ```
> idf.py --version
> ```

Install ESP-IDF v5.5.x (once):

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c2
```

Then, **in every shell** where you build, source the environment:

```sh
. ~/esp/esp-idf/export.sh
```

(If you already have an IDF checkout, `cd ~/esp/esp-idf && git fetch &&
git checkout v5.5.2 && git submodule update --init --recursive &&
./install.sh esp32c2` gets you onto the right version.)

---

## 2. Set the target

From the firmware repo root, once per clean checkout:

```sh
idf.py set-target esp32c2
```

## 3. Choose an sdkconfig (optional)

The repo ships a few defaults:

| file                          | use                                                        |
|-------------------------------|------------------------------------------------------------|
| `sdkconfig.defaults`          | **default — use this.** Includes the GigaDevice flash driver. |
| `sdkconfig.defaults.mini`     | smaller build (trimmed features)                           |
| `sdkconfig.defaults.dram_optimised` | DRAM-tight build. Uses the ROM flash driver and **disables the GD chip driver** — avoid on GD-flash modules unless you know you need it. |

`idf.py` applies `sdkconfig.defaults` automatically. To use a variant, pass it:

```sh
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults" build
```

## 4. Build

```sh
idf.py build
```

The application image you flash/OTA is:

```
build/proxmark5_ble_wifi_module.bin
```

> This is the **app** image (starts with `0xE9`, ESP32-C2 `chip_id 0x000C`).
> It is what OTA and `app_a`/`app_b` expect — **not** the combined/merged
> `*-merged.bin` that esptool writes at offset 0x0. `hw bwm upgrade` validates
> this and refuses anything else.

Optional: set a meaningful version string so `hw status` shows something other
than the IDF default `1`:

```sh
idf.py -D PROJECT_VER="my-build-1" build
```

---

## Flashing

### A. OTA over the PM5 link (normal, no wires)

From the pm3 client, with the PM5 connected:

```
[usb] pm5 --> hw bwm upgrade -f build/proxmark5_ble_wifi_module.bin
```

- Works on a BWM that boots and responds. Confirm afterwards with `hw status`
  (the `BWM fw version` line) — over USB it re-reads automatically; over
  BLE the reboot drops the link, so reconnect and check.
- Requires a working BWM. It **cannot** revive a fully bricked one — that
  needs the header route below.

### B. Serial via the 5-pin header (provisioning / recovery)

With a USB-UART adapter on the BWM header (BOOT/RXD/TXD/3V3/GND) and the
module held in download mode:

```sh
idf.py -p /dev/ttyUSB0 flash          # build + flash + set partitions
# or, to flash an existing build without rebuilding:
idf.py -p /dev/ttyUSB0 app-flash
```

Watch the console (this is UART0 / GPIO19-20 at 115200):

```sh
idf.py -p /dev/ttyUSB0 monitor        # Ctrl-] to exit
```

The console is invaluable for debugging: a panic backtrace prints here the
instant the firmware faults.

---

## Partition layout (for reference)

The OTA scheme uses two app slots so `hw bwm upgrade` can write the inactive
one and switch to it on reboot:

| partition | type       | size    | purpose                    |
|-----------|------------|---------|----------------------------|
| `nvs`     | data       | 336K    | settings (WiFi, etc.)      |
| `otadata` | data/ota   | 8K      | which app slot to boot     |
| `phy_init`| data/phy   | 4K      | RF calibration             |
| `app_a`   | app/ota_0  | 1856K   | app slot A                 |
| `app_b`   | app/ota_1  | 1856K   | app slot B                 |

`esp_ota_get_next_update_partition()` picks the slot that isn't running, the
image is written there, and `esp_ota_set_boot_partition()` flips `otadata` so
the bootloader loads it next.

---

## Troubleshooting

- **`The component esp_driver_uart could not be found`** → IDF too old. Use
  v5.5.x (see §1).
- **`idf.py: command not found`** → you didn't source `export.sh` in this shell.
- **OTA reports the old version after a successful flash** → the app descriptor
  `version` field defaults to `1` when `PROJECT_VER` isn't set, so both old and
  new builds read `1` and look identical. Set `PROJECT_VER` (see §4) to tell
  builds apart, or verify by build date/behaviour.
- **OTA fails with `-4` on a specific module** → the transfer reached the ESP
  but it went silent (crash/reboot mid-write). Get the panic backtrace from the
  serial console (route B `monitor`) — that's the only way to see the cause.
