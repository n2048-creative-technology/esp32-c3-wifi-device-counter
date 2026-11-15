# XIAO ESP32-C3 / C5 + TFT — Wi-Fi + BLE device counter & relay

Passively estimates how many devices (phones, laptops, wearables) are
nearby by listening to both Wi-Fi and Bluetooth Low Energy, shows the
count, a live list of devices with signal strength and rough distance,
and a live view of the channel scan on the TFT, and drives two outputs:

- **D6** — relay, **ON whenever one or more devices** are present.
- **D7** — **1-second pulse every time the count changes**, up or down.

The screen background flashes **green** when the count goes up and
**red** when it goes down.

One firmware, two boards:

| Board                  | Wi-Fi bands scanned | Channels |
|------------------------|---------------------|----------|
| Seeed XIAO ESP32-C3    | 2.4 GHz             | 14 (1–14) |
| Seeed XIAO ESP32-C5    | 2.4 GHz + 5 GHz     | 42 (1–14, 36–64, 100–144, 149–177) |

Built on the same board + screen as
[`esp32-c3-tft-sdcard-looper-`](https://github.com/n2048-creative-technology/esp32-c3-tft-sdcard-looper-)
(1.8" ST7735 SPI TFT), using D6 and D7 — the two pins that project's
README calls out as free. No SD card is used.

## How the counting works

Both radios only **listen** — the device never joins a network or
transmits on Wi-Fi, and the BLE scan is passive.

### Wi-Fi

The radio runs in promiscuous mode and hops across **every channel the
chip supports**. The list isn't hard-coded per board: at boot the
firmware unlocks all channels (manual country policy — safe because it
never transmits) and keeps every candidate the driver accepts. The
channel list is printed on the serial monitor at boot.

Visiting ~40 channels evenly would make detection slow, so **dwell time
adapts** to what was heard on each channel in the last 30 s
(`ACTIVITY_MEMORY_MS`):

| Channel state                        | Dwell |
|--------------------------------------|-------|
| quiet                                | 110 ms (just over one AP beacon interval) |
| traffic, but no counted clients (APs)| 200 ms |
| counted client devices seen          | 400 ms |

A full sweep takes ~3 s on the C3 and ~6 s on the C5. A Wi-Fi device
stays counted for at least `MIN_SWEEPS_IN_WINDOW` (3) sweeps, so the
effective Wi-Fi window is `max(WIFI_SEEN_WINDOW_MS, 3 × sweep)` —
~15 s on the C3, ~17–20 s on the C5.

To avoid counting the same device more than once, only frames *sent by
client devices* are counted:

- data frames going **to** an access point (including the null-data
  keepalives sleeping phones send) — a connected phone uses one stable
  MAC per network;
- association / reassociation requests;
- probe requests, but **only from non-randomized MACs** (randomized probe
  MACs rotate and would each look like a new device).

Access points (beacons, probe responses, AP → client traffic) are
ignored, so routers — yours or the neighbours' — don't count.

### BLE

A continuous passive scan of advertising packets. A BLE address counts
as present for `BLE_SEEN_WINDOW_MS` (8 s) after its last advert.

### Estimate

Frames weaker than `MIN_RSSI` / `BLE_MIN_RSSI` (-70 dBm) are ignored so
only devices roughly "in the room" count.

**Estimate = max(Wi-Fi count, BLE count).** A phone usually shows up on
both radios with unrelated addresses that can't be linked, so taking the
larger count (not the sum) keeps it from being counted twice.

### Limitations

This is an **estimate**, not an exact headcount:

- **C3 is 2.4 GHz only.** A phone on a 5 GHz network is only seen via BLE
  on the C3. The C5 hears both bands.
- **Hopping.** Each channel is only heard part of the time; a device that
  transmits rarely can be missed for a sweep or two.
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
      …
──────────────────────
ch44 5G  5.8s    BLE ●
▂▂▂▂▂▂▂▂▂▂▂▂▂▂ ▂▂▂█▂▂▂▂
```

- **Header:** estimated count, relay state, Wi-Fi / BLE counts.
- **Device list**, strongest (closest) first: **W** = Wi-Fi, **B** = BLE,
  last two bytes of the address, RSSI (dBm), rough distance, strength
  bar. Colour: green ≥ -50, yellow ≥ -60, orange below. Up to 9 rows,
  then "+N more". Refreshed every `DISPLAY_REFRESH_MS`.
- **Scan strip** (bottom), redrawn on every channel hop:
  - current channel and band (`2G` / `5G`), duration of the last full
    sweep, and a **BLE dot** that lights for 300 ms after any advert;
  - one cell per channel, 2.4 GHz and 5 GHz in separate groups. The
    **tall white** cell is the channel being listened to; **green** =
    counted clients there recently, **blue** = other traffic (APs),
    **dark grey** = quiet.

Both areas are drawn into off-screen buffers and pushed in one go, so
nothing flickers.

Distance uses the log-distance path loss model,
`d = 10^((P1m − RSSI) / (10·n))`. Calibrate by holding a device 1 m away,
reading its RSSI from the serial debug list, and setting
`BLE_RSSI_AT_1M` / `WIFI_RSSI_AT_1M`.

## Outputs

| Output | XIAO pin | Behaviour |
|--------|----------|-----------|
| Relay  | D6       | ON while estimate > `RELAY_THRESHOLD` (default 0 → one or more devices). OFF after the estimate stays at/below it for `RELAY_OFF_DELAY_MS` (default 0 → immediately). |
| Pulse  | D7       | `PULSE_MS` (1 s) pulse on every count change. A change during a running pulse restarts it rather than queueing a second one. |

Set `RELAY_ACTIVE_HIGH` / `PULSE_ACTIVE_HIGH` to match what's connected
(most opto-isolated relay boards are *active-low*: set `false`). Use a
relay module, not a bare coil on the GPIO.

> **Note:** on both boards D6 / D7 are the chip's UART0 TX / RX pins
> (C3: GPIO21/20, C5: GPIO11/12). The ROM bootloader prints boot
> messages on TX at reset, so the relay may click briefly on power-up /
> reset. Move the relay to another free pin if that matters for the load.

## Tunables (`src/main.cpp`, top of file)

| Constant               | Default | Meaning |
|------------------------|---------|---------|
| `WIFI_SEEN_WINDOW_MS`  | 15000   | Minimum time a Wi-Fi address counts as present after its last frame |
| `BLE_SEEN_WINDOW_MS`   | 8000    | Same for BLE (shorter → faster drop, but may flicker) |
| `MIN_SWEEPS_IN_WINDOW` | 3       | Wi-Fi window is stretched to at least this many full sweeps |
| `IDLE_DWELL_MS`        | 110     | Dwell on a quiet channel |
| `AP_DWELL_MS`          | 200     | Dwell on a channel with traffic but no counted clients |
| `BUSY_DWELL_MS`        | 400     | Dwell on a channel with counted clients |
| `ACTIVITY_MEMORY_MS`   | 30000   | How long a channel is remembered as active |
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
| `DISPLAY_REFRESH_MS`   | 500     | Count / list / relay update interval |
| `WIFI_RSSI_AT_1M` / `BLE_RSSI_AT_1M` | -40 / -59 | RSSI at 1 m, for the distance estimate |
| `PATH_LOSS_N`          | 2.5     | Path loss exponent (2 open space, 2.5–3.5 indoors) |
| `DEBUG_LIST`           | true    | Print tracked addresses + RSSI to serial every `DEBUG_LIST_MS` |

## Wiring

The firmware uses the XIAO **pad labels**, so the wiring is identical on
both boards; the board variant maps them to GPIOs.

| Signal       | XIAO pin | C3 GPIO | C5 GPIO |
|--------------|----------|---------|---------|
| TFT VCC      | 3V3      | –       | –       |
| TFT GND      | GND      | –       | –       |
| TFT CS       | D1       | 3       | 0       |
| TFT A0 (DC)  | D2       | 4       | 25      |
| TFT RESET    | D3       | 5       | 7       |
| TFT LED      | D4       | 6       | 23      |
| TFT SCK      | D8       | 8       | 8       |
| TFT SDA      | D10      | 10      | 10      |
| Relay IN     | D6       | 21      | 11      |
| Pulse out    | D7       | 20      | 12      |

Relay VCC: 3V3 or 5V per the module spec. **Logic is 3.3 V only** on the
XIAO's GPIOs — use modules with 3.3 V-compatible inputs.

## Build & upload

Both boards use the [pioarduino](https://github.com/pioarduino/platform-espressif32)
platform (Arduino-ESP32 3.x) — the official `platformio/espressif32`
platform doesn't support the C5. It's pinned to an exact release in
`platformio.ini`; the first build downloads it.

```sh
pio run -e xiao_esp32c3 -t upload    # XIAO ESP32-C3 (default env)
pio run -e xiao_esp32c5 -t upload    # XIAO ESP32-C5
pio device monitor
```

With both boards plugged in, add `--upload-port /dev/ttyACM<n>` to pick
one.

Wi-Fi + the Bluedroid BLE stack don't fit the default 1.25 MB app slot,
so both envs use the `huge_app.csv` partition table (3 MB app, no OTA).

The serial monitor prints the channel list at boot:

```
scanning 42 channels (14 x 2.4GHz, 28 x 5GHz): 1 2 3 … 173 177
```

then, every `DISPLAY_REFRESH_MS`:

```
estimate: 1 (wifi 1, ble 1)  relay: ON  raw pkts: 128  raw adverts: 73  ch 56  sweep 5790ms  wifi window 17370ms
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
