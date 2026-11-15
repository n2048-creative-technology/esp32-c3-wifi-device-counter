# XIAO ESP32-C3 + TFT — Wi-Fi + BLE device counter & relay

Passively estimates how many devices (phones, laptops, wearables) are
nearby by listening to both Wi-Fi and Bluetooth Low Energy, shows the
count and a live list of devices with signal strength and rough distance
on the TFT, and drives two outputs:

- **D6** — relay, **ON whenever one or more devices** are present.
- **D7** — **1-second pulse every time the count changes**, up or down.

The screen background flashes **green** when the count goes up and
**red** when it goes down.

Built on the same board + screen as
[`esp32-c3-tft-sdcard-looper-`](https://github.com/n2048-creative-technology/esp32-c3-tft-sdcard-looper-)
(Seeed XIAO ESP32-C3, 1.8" ST7735 SPI TFT), using D6 and D7 — the two
pins that project's README calls out as free. No SD card is used.

## How the counting works

Both radios only **listen** — the device never joins a network or
transmits.

**Wi-Fi** — the radio runs in promiscuous mode, hopping across 2.4 GHz
channels 1 / 6 / 11. To avoid counting the same device more than once,
only frames *sent by client devices* are counted:

- data frames going **to** an access point (including the null-data
  keepalives sleeping phones send) — a connected phone uses one stable
  MAC per network;
- association / reassociation requests;
- probe requests, but **only from non-randomized MACs** (randomized probe
  MACs rotate and would each look like a new device).

Access points (beacons, probe responses, AP → client traffic) are
ignored, so routers — yours or the neighbours' — don't count.

**BLE** — a continuous passive scan of advertising packets.

Each radio keeps its own table of addresses with a last-seen time. An
address counts as present for `WIFI_SEEN_WINDOW_MS` (15 s) /
`BLE_SEEN_WINDOW_MS` (8 s) after its last packet, then expires — that's
what makes the count drop shortly after a device leaves or turns its
radios off. Frames weaker than `MIN_RSSI` / `BLE_MIN_RSSI` (-70 dBm) are
ignored so only devices roughly "in the room" count.

**Estimate = max(Wi-Fi count, BLE count).** A phone usually shows up on
both radios with unrelated addresses that can't be linked, so taking the
larger count (not the sum) keeps it from being counted twice.

### Limitations

This is an **estimate**, not an exact headcount:

- **2.4 GHz only.** The ESP32-C3 can't hear 5 GHz Wi-Fi; a phone on a
  5 GHz network is only seen via BLE.
- **BLE address rotation.** Phones change their BLE address roughly every
  15 minutes; for up to `BLE_SEEN_WINDOW_MS` the old and new address
  overlap, so the count can briefly go +1 and back.
- **Fixed devices count too** (TVs, speakers, smart plugs, a neighbour's
  laptop through the wall). Add stable addresses to `IGNORE_MACS`.
- **Idle devices** that aren't transmitting aren't seen until they next do.
- **Distance is rough** — walls, bodies and phone orientation easily
  shift RSSI by 10 dB. Treat it as near / medium / far.

## Display

```
devices      relay ON
 2           W1 B2
──────────────────────
B ed:76 -55 ~0.6m ████
W 0e:be -62 ~7.6m ██
B 2a:1f -68 ~2.3m █
```

- Top: estimated count, relay state, Wi-Fi / BLE counts.
- List, strongest (closest) first: **W** = Wi-Fi, **B** = BLE, last two
  bytes of the address, RSSI (dBm), rough distance, strength bar. Colour:
  green ≥ -50, yellow ≥ -60, orange below. Up to 11 rows, then "+N more".
- Frames are drawn into an off-screen buffer and pushed in one go, so
  the list doesn't flicker.

Distance uses the log-distance path loss model,
`d = 10^((P1m − RSSI) / (10·n))`. Calibrate by holding a device 1 m away,
reading its RSSI from the serial debug list, and setting
`BLE_RSSI_AT_1M` / `WIFI_RSSI_AT_1M`.

## Outputs

| Output | XIAO pin | GPIO   | Behaviour |
|--------|----------|--------|-----------|
| Relay  | D6       | GPIO21 | ON while estimate > `RELAY_THRESHOLD` (default 0 → one or more devices). OFF after the estimate stays at/below it for `RELAY_OFF_DELAY_MS` (default 0 → immediately). |
| Pulse  | D7       | GPIO20 | `PULSE_MS` (1 s) pulse on every count change. A change during a running pulse restarts it rather than queueing a second one. |

Set `RELAY_ACTIVE_HIGH` / `PULSE_ACTIVE_HIGH` to match what's connected
(most opto-isolated relay boards are *active-low*: set `false`). Use a
relay module, not a bare coil on the GPIO.

> **Note:** GPIO21 (D6) is also the chip's UART0 TX pin, and the ROM
> bootloader prints boot messages on it at reset. The relay may click
> briefly on power-up / reset. Move the relay to another free pin if
> that matters for the load.

## Tunables (`src/main.cpp`, top of file)

| Constant               | Default | Meaning |
|------------------------|---------|---------|
| `WIFI_SEEN_WINDOW_MS`  | 15000   | Wi-Fi address counts as present this long after last frame |
| `BLE_SEEN_WINDOW_MS`   | 8000    | Same for BLE (shorter → faster drop, but may flicker) |
| `CHANNEL_DWELL_MS`     | 300     | Time per Wi-Fi channel before hopping |
| `CHANNELS`             | 1,6,11  | 2.4 GHz channels scanned — add your router's if it's elsewhere |
| `MAX_TRACKED_MACS`     | 128     | Tracking table size, per radio |
| `MIN_RSSI` / `BLE_MIN_RSSI` | -70 | Ignore weaker frames (dBm) — sets the "in the room" range |
| `IGNORE_MACS`          | –       | Addresses never counted or listed (fixed devices) |
| `RELAY_THRESHOLD`      | 0       | Relay ON when estimate is greater than this |
| `RELAY_OFF_DELAY_MS`   | 0       | Hold-off before the relay turns OFF |
| `RELAY_ACTIVE_HIGH`    | true    | `false` for active-low relay boards |
| `PULSE_MS`             | 1000    | D7 pulse length |
| `PULSE_ACTIVE_HIGH`    | true    | `false` for an active-low D7 input |
| `FLASH_MS`             | 400     | Background flash length on count change |
| `FLASH_UP_BG` / `FLASH_DOWN_BG` | dark green / dark red | Flash colours (RGB565) |
| `BACKLIGHT_LEVEL`      | 200     | Backlight PWM (0–255) |
| `DISPLAY_REFRESH_MS`   | 500     | Screen / relay update interval |
| `WIFI_RSSI_AT_1M` / `BLE_RSSI_AT_1M` | -40 / -59 | RSSI at 1 m, for the distance estimate |
| `PATH_LOSS_N`          | 2.5     | Path loss exponent (2 open space, 2.5–3.5 indoors) |
| `DEBUG_LIST`           | true    | Print tracked addresses + RSSI to serial every `DEBUG_LIST_MS` |

## Wiring

TFT (same as the looper project, SD card lines not needed):

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

Outputs:

| Signal         | XIAO pin | GPIO   |
|----------------|----------|--------|
| Relay IN       | D6       | GPIO21 |
| Pulse out      | D7       | GPIO20 |
| Relay VCC      | 3V3 or 5V (per module spec) | – |
| GND            | GND      | –      |

**Logic is 3.3 V only** on the XIAO's GPIOs — use modules with
3.3 V-compatible inputs.

## Build & upload

```sh
pio run -t upload
pio device monitor
```

Wi-Fi + the Bluedroid BLE stack don't fit the default 1.25 MB app slot,
so `platformio.ini` uses the `huge_app.csv` partition table (3 MB app,
no OTA).

The serial monitor prints, every `DISPLAY_REFRESH_MS`:

```
estimate: 2 (wifi 1, ble 2)  relay: ON  raw pkts: 672  raw adverts: 210
```

plus a `count changed X -> Y` line on every change and, with
`DEBUG_LIST`, the full tracked list with RSSI every 5 s. `raw pkts` /
`raw adverts` count everything the radios receive before filtering —
if they stay at 0 the radio isn't running at all.

### Tuning tips

1. Turn `DEBUG_LIST` on, walk around with your phone, and pick
   `MIN_RSSI` / `BLE_MIN_RSSI` for the range you care about.
2. With your own devices off, anything still listed is a fixed device —
   add its address (as printed) to `IGNORE_MACS` if it's stable.
3. Turn `DEBUG_LIST` off when done.
