# XIAO ESP32-C3 + TFT — Wi-Fi device counter & relay

Passively estimates how many Wi-Fi devices (phones, laptops, anything
transmitting 802.11) are nearby and drives a relay ON whenever that
estimate is **greater than one**. Built on the same board + screen as
[`esp32-c3-tft-sdcard-looper-`](https://github.com/n2048-creative-technology/esp32-c3-tft-sdcard-looper-)
(Seeed XIAO ESP32-C3, 1.8" ST7735 SPI TFT), reusing D6 and D7 — the two
pins that project's README calls out as free.

No SD card is used here; the TFT just shows a live number.

## How the counting works

The ESP32-C3's Wi-Fi radio is put into **promiscuous mode** (sniffing
only — it never joins or transmits on any network) and hops across
channels 1 / 6 / 11. Every unique source MAC address seen in management
or data frames is tracked with a last-seen timestamp; a MAC counts as
"present" for `SEEN_WINDOW_MS` (2 minutes by default) after its last
packet, then expires. The displayed number is the count of currently
non-expired MACs.

This is an **estimate**, the same kind used by commercial Wi-Fi people
counters, with two known biases:

- **Over-counting**: iOS/Android MAC address randomization means an idle
  phone can appear as several different "devices" over time as it sends
  probe requests with rotating random MACs.
- **Under-counting**: a device that's asleep/idle and not transmitting
  anything won't be seen at all until it next sends a frame.

Range is typically tens of metres depending on antenna and obstacles.
This is good enough for "is anyone here vs. is it just me" logic, not
for exact headcounts.

## Relay behaviour

D6 (GPIO21) drives a relay module (opto-isolated relay board recommended
— do not drive a bare relay coil straight from the GPIO):

- Estimated count > `RELAY_THRESHOLD` (default `1`) → relay **ON**
- Otherwise → relay **OFF**

Set `RELAY_ACTIVE_HIGH` to match your relay module (most common opto
relay boards are *active-low*: set it to `false`). D7 (GPIO20) is left
free for a second relay or a status input.

## Tunables (`src/main.cpp`, top of file)

| Constant              | Default | Meaning                                           |
|------------------------|---------|---------------------------------------------------|
| `SEEN_WINDOW_MS`       | 120000  | How long a MAC still counts as "present"          |
| `CHANNEL_DWELL_MS`     | 300     | Time spent per channel before hopping             |
| `CHANNELS`             | 1,6,11  | 2.4GHz channels scanned (non-overlapping)         |
| `MAX_TRACKED_MACS`     | 128     | Size of the tracking table                        |
| `RELAY_THRESHOLD`      | 1       | Relay turns ON when count is greater than this    |
| `RELAY_ACTIVE_HIGH`    | true    | Flip to `false` for active-low relay boards       |
| `DISPLAY_REFRESH_MS`   | 500     | Screen/relay update interval                      |
| `MIN_RSSI`             | -90     | Ignore frames weaker than this (noise floor, dBm) |

All tunables are small named constants, no rebuild-heavy config system —
edit, reflash, done.

## Wiring

Same TFT wiring as the looper project (SD card lines not needed):

| Display pin | XIAO pin | GPIO   |
|-------------|----------|--------|
| VCC         | 3V3      | –      |
| GND         | GND      | –      |
| CS          | D1       | GPIO3  |
| RESET       | D3       | GPIO5  |
| A0 (DC)     | D2       | GPIO4  |
| SDA (MOSI)  | D10      | GPIO10 |
| SCK         | D8       | GPIO8  |
| LED         | D4       | GPIO6  |

Relay:

| Relay pin | XIAO pin | GPIO   |
|-----------|----------|--------|
| IN        | D6       | GPIO21 |
| VCC       | 3V3 or 5V (per relay module spec) | – |
| GND       | GND      | –      |

**Logic is 3.3V only** on the XIAO's GPIOs — use a relay module with a
3.3V-compatible logic input (most opto-isolated boards are fine).

## Build & upload

```sh
pio run -t upload
pio device monitor
```

The serial monitor prints the estimated count and relay state once per
`DISPLAY_REFRESH_MS`.

## Known limitations / ideas for improvement

- No persistence — counts reset on reboot (by design, cheap and stateless).
- Could add BLE scanning (XIAO C3 has BLE too) to also catch
  BLE-only devices (wearables, earbuds) for a slightly less Wi-Fi-biased
  estimate.
- Could expose the count over Wi-Fi (a small HTTP `/status` JSON
  endpoint) for remote/dashboard monitoring — currently local-display
  only by design to keep this project minimal.
