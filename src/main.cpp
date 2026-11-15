// XIAO ESP32-C3 + 128x160 ST7735 SPI TFT — ambient Wi-Fi + BLE device
// counter with a relay output.
//
// Two passive sources, each tracked in its own table of unique addresses
// seen in a rolling time window:
//   - Wi-Fi: promiscuous sniffing (never joins a network), hopping across
//     channels 1/6/11, 2.4 GHz only. Only CLIENT-sent frames are counted:
//     data frames going to an access point (stable per-network MAC), plus
//     association requests and probe requests from non-randomized MACs.
//     Access points (beacons, probe responses, AP->client data) and
//     randomized probe MACs are ignored, so routers and rotating probe
//     addresses don't inflate the count.
//   - BLE:   continuous passive scan of advertising packets.
// It is an ESTIMATE, not an exact count:
//   - iOS/Android randomize Wi-Fi probe MACs and BLE addresses, so one
//     phone can look like several devices over time (tends to OVER-count).
//   - Idle devices that aren't transmitting won't be seen (UNDER-count).
//   - Phones not connected to any 2.4 GHz network are only seen via BLE.
//   - The same phone shows up on BOTH radios with unrelated addresses, so
//     the two can't be de-duplicated. The estimate is max(wifi, ble), so a
//     phone seen on both still counts once.
//   - Fixed devices (smart plugs, TVs, a neighbour's laptop) count like any
//     other; put their addresses in IGNORE_MACS.
//
// The estimate is shown on the TFT. D6 (GPIO21) drives a relay: ON while
// the estimate exceeds RELAY_THRESHOLD (default 0, i.e. one or more
// devices), OFF once it has stayed at or below it for RELAY_OFF_DELAY_MS
// (default 0 = immediately). D7 (GPIO20) gives a PULSE_MS pulse every time
// the estimate changes, up or down.
//
// Based on n2048-creative-technology/esp32-c3-tft-sdcard-looper- (same
// board + screen). The SD card and its SPI lines are not used here.

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <algorithm>

// ---- Pin map (XIAO label -> GPIO), matching the looper project ----------
constexpr int PIN_SCK     = 8;   // D8  -> TFT SCK
constexpr int PIN_MOSI    = 10;  // D10 -> TFT SDA (MOSI)
constexpr int PIN_TFT_CS  = 3;   // D1  -> CS
constexpr int PIN_TFT_DC  = 4;   // D2  -> A0 (data/command)
constexpr int PIN_TFT_RST = 5;   // D3  -> RESET
constexpr int PIN_TFT_LED = 6;   // D4  -> LED (backlight, PWM)
constexpr int PIN_RELAY   = 21;  // D6  -> relay module IN (free pin on the looper board)
constexpr int PIN_PULSE   = 20;  // D7  -> pulse output on every count change

// ---- Tunables -------------------------------------------------------------
// How long an address still counts as "present" after its last frame —
// this is the main "how fast does it drop" knob. Phones advertise BLE many
// times per second, so BLE can be short; connected-but-sleeping phones only
// send Wi-Fi keepalives every few seconds, so Wi-Fi needs a bit longer.
constexpr uint32_t WIFI_SEEN_WINDOW_MS = 15000;
constexpr uint32_t BLE_SEEN_WINDOW_MS  = 8000;
constexpr uint32_t CHANNEL_DWELL_MS = 300;     // time spent per Wi-Fi channel before hopping
constexpr uint8_t  CHANNELS[]       = {1, 6, 11};  // non-overlapping 2.4GHz channels
constexpr size_t   MAX_TRACKED_MACS = 128;     // table size, per radio
constexpr uint8_t  RELAY_THRESHOLD  = 0;        // relay ON when estimate > this (0 = one or more devices)
constexpr uint32_t RELAY_OFF_DELAY_MS = 0;      // estimate must stay <= threshold this long before relay goes OFF
constexpr bool     RELAY_ACTIVE_HIGH = true;    // flip to false for active-low relay modules
constexpr uint32_t PULSE_MS          = 1000;    // D7 pulse length on each count change
constexpr bool     PULSE_ACTIVE_HIGH = true;    // flip to false if D7 drives an active-low input
constexpr uint8_t  BACKLIGHT_LEVEL   = 200;     // backlight PWM (0-255)
constexpr uint32_t FLASH_MS          = 400;     // background flash length on each count change
constexpr uint16_t FLASH_UP_BG       = 0x0480;  // dark green (RGB565): count went up
constexpr uint16_t FLASH_DOWN_BG     = 0x9000;  // dark red (RGB565): count went down
constexpr uint32_t DISPLAY_REFRESH_MS = 500;    // how often the screen number updates
constexpr int8_t   MIN_RSSI          = -70;     // Wi-Fi: ignore frames weaker than this (dBm) — "in the room"
constexpr int8_t   BLE_MIN_RSSI      = -70;     // BLE: ignore adverts weaker than this (dBm)
constexpr bool     DEBUG_LIST        = true;    // print tracked addresses + RSSI every DEBUG_LIST_MS (for tuning)
constexpr uint32_t DEBUG_LIST_MS     = 5000;

// Addresses never counted or listed, e.g. fixed devices that are always in
// range. Copy them from the serial debug list ("wifi aa:bb:..." / "ble ...").
// Phones' BLE addresses rotate every ~15 min, so only ignore stable ones.
// (The all-zero entry is a placeholder and never matches.)
constexpr uint8_t IGNORE_MACS[][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  // {0x2c, 0x9f, 0xfb, 0xda, 0x0e, 0xbe},  // e.g. the always-present Wi-Fi device at -70 dBm
};

// Rough RSSI -> distance (log-distance path loss): d = 10^((P1m - rssi) / (10 n)).
// P1m = RSSI measured at 1 m; calibrate by holding a device 1 m away and
// reading its RSSI from the debug list. Indoors this is +-50% at best.
constexpr float WIFI_RSSI_AT_1M = -40.0f;  // phones transmit Wi-Fi fairly hot
constexpr float BLE_RSSI_AT_1M  = -59.0f;  // typical phone BLE advert
constexpr float PATH_LOSS_N     = 2.5f;    // 2 = open space, 2.5-3.5 indoors

constexpr uint32_t TFT_SPI_HZ = 27000000;

Adafruit_ST7735 tft(&SPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
// Off-screen frame buffer (128x160x2 = 40 KB): draw everything here, then
// push it in one go, so the list doesn't flicker on each refresh.
GFXcanvas16 canvas(128, 160);

// ---- Address tracking table (fixed-size, no heap churn in callbacks) -----
// Written from the Wi-Fi / BT driver tasks, swept from loop(); a spinlock
// keeps the two from tearing an entry.
struct SeenMac {
  uint8_t mac[6] = {0};
  uint32_t lastSeenMs = 0;
  int8_t rssi = 0;
  bool used = false;
};

struct ListEntry {  // one row of the on-screen device list
  uint8_t mac[6];
  int8_t rssi;
  bool ble;
};

struct Tracker {
  const uint32_t windowMs;
  SeenMac slots[MAX_TRACKED_MACS];
  explicit Tracker(uint32_t window) : windowMs(window) {}
  portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

  void IRAM_ATTR touch(const uint8_t *mac, int8_t rssi) {
    uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    int freeSlot = -1;
    for (size_t i = 0; i < MAX_TRACKED_MACS; i++) {
      if (slots[i].used && memcmp(slots[i].mac, mac, 6) == 0) {
        slots[i].lastSeenMs = now;
        // Light smoothing so the shown power/distance doesn't jump around.
        slots[i].rssi = (int8_t)((3 * slots[i].rssi + rssi) / 4);
        portEXIT_CRITICAL(&lock);
        return;
      }
      if (!slots[i].used && freeSlot < 0) freeSlot = (int)i;
    }
    if (freeSlot >= 0) {  // table full -> drop (undercount under heavy load)
      memcpy(slots[freeSlot].mac, mac, 6);
      slots[freeSlot].lastSeenMs = now;
      slots[freeSlot].rssi = rssi;
      slots[freeSlot].used = true;
    }
    portEXIT_CRITICAL(&lock);
  }

  // Expires stale entries and returns the live count.
  uint16_t countActive() {
    uint32_t now = millis();
    uint16_t count = 0;
    portENTER_CRITICAL(&lock);
    for (size_t i = 0; i < MAX_TRACKED_MACS; i++) {
      if (!slots[i].used) continue;
      if (now - slots[i].lastSeenMs > windowMs) {
        slots[i].used = false;
        continue;
      }
      count++;
    }
    portEXIT_CRITICAL(&lock);
    return count;
  }

  // Appends live entries to out[n..max), returns the new n.
  size_t snapshot(ListEntry *out, size_t n, size_t max, bool ble) {
    uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    for (size_t i = 0; i < MAX_TRACKED_MACS && n < max; i++) {
      if (!slots[i].used || now - slots[i].lastSeenMs > windowMs) continue;
      memcpy(out[n].mac, slots[i].mac, 6);
      out[n].rssi = slots[i].rssi;
      out[n].ble = ble;
      n++;
    }
    portEXIT_CRITICAL(&lock);
    return n;
  }

  void printActive(const char *label) {
    SeenMac copy[MAX_TRACKED_MACS];
    portENTER_CRITICAL(&lock);
    memcpy(copy, slots, sizeof(copy));
    portEXIT_CRITICAL(&lock);
    uint32_t now = millis();
    for (const auto &e : copy) {
      if (!e.used) continue;
      Serial.printf("  %s %02x:%02x:%02x:%02x:%02x:%02x  rssi %4d  seen %2lus ago\n", label,
                    e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5],
                    e.rssi, (unsigned long)((now - e.lastSeenMs) / 1000));
    }
  }
};

Tracker wifiSeen(WIFI_SEEN_WINDOW_MS);
Tracker bleSeen(BLE_SEEN_WINDOW_MS);
volatile uint32_t rawPackets = 0;  // diagnostics: every frame the Wi-Fi callback sees
volatile uint32_t rawAdverts = 0;  // diagnostics: every BLE advert the scan callback sees

static bool IRAM_ATTR isIgnored(const uint8_t *mac) {
  for (const auto &m : IGNORE_MACS)
    if (memcmp(m, mac, 6) == 0) return true;
  return false;
}

static void checkErr(const char *what, esp_err_t err) {
  if (err != ESP_OK) Serial.printf("%s failed: %s\n", what, esp_err_to_name(err));
}

// ---- Wi-Fi sniffer --------------------------------------------------------
// Called from the Wi-Fi driver's own task — keep this fast and allocation-free.
static void IRAM_ATTR onPacket(void *buf, wifi_promiscuous_pkt_type_t type) {
  rawPackets++;
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  auto *pkt = (wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.rssi < MIN_RSSI) return;
  if (pkt->rx_ctrl.sig_len < 16) return;

  // 802.11 frame control: byte 0 = subtype(7..4) type(3..2), byte 1 bit0 =
  // ToDS, bit1 = FromDS. addr2 (byte 10) is the transmitter address.
  const uint8_t *p = pkt->payload;
  const uint8_t ftype   = (p[0] >> 2) & 0x03;
  const uint8_t subtype = (p[0] >> 4) & 0x0F;
  const bool toDS   = p[1] & 0x01;
  const bool fromDS = p[1] & 0x02;
  const uint8_t *srcMac = p + 10;
  const bool randomMac = srcMac[0] & 0x02;  // locally-administered bit

  bool clientFrame = false;
  if (ftype == 2) {
    // Data (incl. null-data keepalives from sleeping phones): only
    // client -> AP. Excludes AP -> client and mesh/WDS (both bits set).
    clientFrame = toDS && !fromDS;
  } else if (ftype == 0) {
    switch (subtype) {
      case 0:  // association request
      case 2:  // reassociation request
        clientFrame = true;
        break;
      case 4:  // probe request: only non-randomized MACs, rotating ones
        clientFrame = !randomMac;  // would each count as a new device
        break;
      default:  // beacons, probe responses, auth, action... -> skip
        break;
    }
  }
  if (!clientFrame) return;

  // Ignore broadcast/multicast and all-zero addresses (malformed/noise).
  if (srcMac[0] & 0x01) return;
  bool allZero = true;
  for (int i = 0; i < 6; i++) if (srcMac[i] != 0) { allZero = false; break; }
  if (allZero) return;

  if (isIgnored(srcMac)) return;
  wifiSeen.touch(srcMac, pkt->rx_ctrl.rssi);
}

static void setupSniffer() {
  // Promiscuous mode needs the Wi-Fi driver started. NOTE: in Arduino,
  // WIFI_MODE_NULL == WIFI_OFF and de-inits the driver, so use STA mode
  // and simply never connect.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
  };
  checkErr("set_promiscuous_filter", esp_wifi_set_promiscuous_filter(&filter));
  checkErr("set_promiscuous_rx_cb", esp_wifi_set_promiscuous_rx_cb(&onPacket));
  checkErr("set_promiscuous", esp_wifi_set_promiscuous(true));
  checkErr("set_channel", esp_wifi_set_channel(CHANNELS[0], WIFI_SECOND_CHAN_NONE));
}

static void hopChannel() {
  static size_t idx = 0;
  idx = (idx + 1) % (sizeof(CHANNELS) / sizeof(CHANNELS[0]));
  esp_wifi_set_channel(CHANNELS[idx], WIFI_SECOND_CHAN_NONE);
}

// ---- BLE scanner ----------------------------------------------------------
class AdvertCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    rawAdverts++;
    if (!dev.haveRSSI() || dev.getRSSI() < BLE_MIN_RSSI) return;
    // BLEAddress stores bytes in display order; uniqueness is all we need.
    const uint8_t *mac = *dev.getAddress().getNative();
    if (isIgnored(mac)) return;
    bleSeen.touch(mac, (int8_t)dev.getRSSI());
  }
};

static BLEScan *bleScan = nullptr;

static void onScanComplete(BLEScanResults) {
  // Shouldn't happen with duration 0, but restart if the stack ever stops.
  bleScan->start(0, onScanComplete, false);
}

static void setupBle() {
  BLEDevice::init("");
  bleScan = BLEDevice::getScan();
  // wantDuplicates=true: we get every advert (keeps lastSeen fresh) and
  // the library doesn't accumulate results in RAM.
  bleScan->setAdvertisedDeviceCallbacks(new AdvertCallbacks(), true);
  bleScan->setActiveScan(false);  // passive: listen only, never transmit
  bleScan->setInterval(100);
  bleScan->setWindow(50);         // leave radio time for the Wi-Fi sniffer
  if (!bleScan->start(0, onScanComplete, false)) {
    Serial.println("BLE scan start failed");
  }
}

// ---- Display --------------------------------------------------------------
static float rssiToMeters(int8_t rssi, bool ble) {
  float p1m = ble ? BLE_RSSI_AT_1M : WIFI_RSSI_AT_1M;
  return powf(10.0f, (p1m - rssi) / (10.0f * PATH_LOSS_N));
}

static uint16_t rssiColor(int8_t rssi) {
  if (rssi >= -50) return ST77XX_GREEN;
  if (rssi >= -60) return ST77XX_YELLOW;
  return ST77XX_ORANGE;
}

// bg = background colour: black normally, green/red while flashing.
static void drawScreen(uint16_t estimate, uint16_t wifi, uint16_t ble, bool relayOn,
                       uint16_t bg) {
  static ListEntry list[2 * MAX_TRACKED_MACS];
  size_t n = wifiSeen.snapshot(list, 0, 2 * MAX_TRACKED_MACS, false);
  n = bleSeen.snapshot(list, n, 2 * MAX_TRACKED_MACS, true);
  std::sort(list, list + n, [](const ListEntry &a, const ListEntry &b) {
    return a.rssi > b.rssi;  // strongest (closest) first
  });

  GFXcanvas16 &c = canvas;
  c.fillScreen(bg);
  const bool flash = bg != ST77XX_BLACK;

  // Header: label, big count, relay + per-radio counts on the right.
  c.setTextSize(1);
  c.setTextColor(ST77XX_WHITE);
  c.setCursor(2, 2);
  c.print("devices");
  c.setTextSize(3);
  // White while flashing so the number stays readable on green/red.
  c.setTextColor(flash ? ST77XX_WHITE
                       : (estimate > RELAY_THRESHOLD ? ST77XX_GREEN : ST77XX_YELLOW));
  c.setCursor(2, 13);
  c.print(min(estimate, (uint16_t)999));
  c.setTextSize(1);
  c.setTextColor(relayOn ? ST77XX_GREEN : 0x7BEF);
  c.setCursor(64, 14);
  c.print(relayOn ? "relay ON" : "relay off");
  c.setTextColor(ST77XX_WHITE);
  c.setCursor(64, 26);
  c.printf("W%u B%u", wifi, ble);
  c.drawFastHLine(0, 40, 128, 0x7BEF);

  // List: "W 0e:be -62 ~1.8m [bar]", strongest first. W = Wi-Fi, B = BLE.
  constexpr int ROW_Y0 = 44, ROW_H = 10;
  const size_t rows = (160 - ROW_Y0) / ROW_H;
  if (n == 0) {
    c.setTextColor(0x7BEF);
    c.setCursor(2, ROW_Y0);
    c.print("nothing in range");
  }
  for (size_t i = 0; i < n && i < rows; i++) {
    int y = ROW_Y0 + (int)i * ROW_H;
    if (i == rows - 1 && n > rows) {
      c.setTextColor(0x7BEF);
      c.setCursor(2, y);
      c.printf("+%u more", (unsigned)(n - rows + 1));
      break;
    }
    const ListEntry &e = list[i];
    c.setTextColor(e.ble ? ST77XX_MAGENTA : ST77XX_CYAN);
    c.setCursor(2, y);
    c.print(e.ble ? 'B' : 'W');
    c.setTextColor(ST77XX_WHITE);
    c.setCursor(12, y);
    c.printf("%02x:%02x", e.mac[4], e.mac[5]);  // last 2 bytes as a short ID
    uint16_t col = rssiColor(e.rssi);
    c.setTextColor(col);
    c.setCursor(46, y);
    c.printf("%d", e.rssi);
    float m = rssiToMeters(e.rssi, e.ble);
    c.setCursor(70, y);
    if (m < 10) c.printf("~%.1fm", m); else c.printf("~%um", (unsigned)min(m, 99.0f));
    // Strength bar: 0 px at the cutoff, full at -30 dBm.
    int8_t floorDb = e.ble ? BLE_MIN_RSSI : MIN_RSSI;
    int w = constrain((e.rssi - floorDb) * 18 / (-30 - floorDb), 1, 18);
    c.fillRect(108, y, w, 7, col);
  }

  tft.drawRGBBitmap(0, 0, c.getBuffer(), c.width(), c.height());
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_HIGH ? LOW : HIGH);  // start OFF
  pinMode(PIN_PULSE, OUTPUT);
  digitalWrite(PIN_PULSE, PULSE_ACTIVE_HIGH ? LOW : HIGH);  // start idle

  pinMode(PIN_TFT_LED, OUTPUT);
  analogWrite(PIN_TFT_LED, BACKLIGHT_LEVEL);

  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_TFT_CS);
  SPI.setFrequency(TFT_SPI_HZ);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);

  setupSniffer();
  setupBle();
  Serial.println("Device counter started (Wi-Fi sniffer + BLE scan).");
}

void loop() {
  static uint32_t lastHop = 0, lastDraw = 0, lastList = 0, lastAboveMs = 0;
  static uint32_t pulseStartMs = 0;
  static bool relayOn = false, pulsing = false, flashing = false;
  static uint16_t flashBg = ST77XX_BLACK;
  static uint16_t lastEstimate = 0;
  uint32_t now = millis();

  if (now - lastHop >= CHANNEL_DWELL_MS) {
    lastHop = now;
    hopChannel();
  }

  // End the D7 pulse once it has lasted PULSE_MS.
  if (pulsing && now - pulseStartMs >= PULSE_MS) {
    pulsing = false;
    digitalWrite(PIN_PULSE, PULSE_ACTIVE_HIGH ? LOW : HIGH);
  }
  // End the background flash: force an immediate redraw in black.
  if (flashing && now - pulseStartMs >= FLASH_MS) {
    flashing = false;
    flashBg = ST77XX_BLACK;
    lastDraw = now - DISPLAY_REFRESH_MS;
  }

  if (now - lastDraw >= DISPLAY_REFRESH_MS) {
    lastDraw = now;
    uint16_t wifi = wifiSeen.countActive();
    uint16_t ble = bleSeen.countActive();
    // A phone usually shows on both radios with unrelated addresses, so
    // take the larger count rather than the sum.
    uint16_t estimate = max(wifi, ble);
    if (estimate != lastEstimate) {
      // Count changed (up or down): (re)start a PULSE_MS pulse on D7. A
      // change during a running pulse restarts it rather than queueing.
      Serial.printf("count changed %u -> %u: pulse D7\n", lastEstimate, estimate);
      flashBg = estimate > lastEstimate ? FLASH_UP_BG : FLASH_DOWN_BG;
      flashing = true;
      lastEstimate = estimate;
      pulsing = true;
      pulseStartMs = now;
      digitalWrite(PIN_PULSE, PULSE_ACTIVE_HIGH ? HIGH : LOW);
    }
    if (estimate > RELAY_THRESHOLD) {
      relayOn = true;  // ON immediately
      lastAboveMs = now;
    } else if (relayOn && now - lastAboveMs >= RELAY_OFF_DELAY_MS) {
      relayOn = false;
    }
    // XOR: active-high board -> HIGH means ON; active-low board -> LOW means ON.
    digitalWrite(PIN_RELAY, relayOn == RELAY_ACTIVE_HIGH ? HIGH : LOW);
    drawScreen(estimate, wifi, ble, relayOn, flashBg);
    Serial.printf("estimate: %u (wifi %u, ble %u)  relay: %s  raw pkts: %lu  raw adverts: %lu\n",
                  estimate, wifi, ble, relayOn ? "ON" : "off",
                  (unsigned long)rawPackets, (unsigned long)rawAdverts);
  }

  if (DEBUG_LIST && now - lastList >= DEBUG_LIST_MS) {
    lastList = now;
    Serial.println("-- tracked --");
    wifiSeen.printActive("wifi");
    bleSeen.printActive("ble ");
  }
}
