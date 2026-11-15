// Seeed XIAO ESP32-C3 / XIAO ESP32-C5 + 128x160 ST7735 SPI TFT — ambient
// Wi-Fi + BLE device counter with relay and pulse outputs.
//
// Two passive sources, each tracked in its own table of unique addresses
// seen in a rolling time window:
//   - Wi-Fi: promiscuous sniffing (never joins a network), hopping across
//     EVERY channel the chip supports: 2.4 GHz 1-14 on both boards, plus all
//     5 GHz channels (36-177) on the C5. The list is discovered at boot by
//     asking the driver which channels it accepts. Dwell time adapts: quiet
//     channels get a short look (enough to catch an AP beacon), channels
//     with recent traffic get longer, so a full sweep stays a few seconds
//     even with ~40 channels. Only CLIENT-sent frames are counted:
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
//   - On the C3, phones on 5 GHz networks are only seen via BLE.
//   - Hopping means each channel is only heard part of the time; a device
//     that transmits rarely can be missed for a sweep or two.
//   - The same phone shows up on BOTH radios with unrelated addresses, so
//     the two can't be de-duplicated. The estimate is max(wifi, ble), so a
//     phone seen on both still counts once.
//   - Fixed devices (smart plugs, TVs, a neighbour's laptop) count like any
//     other; put their addresses in IGNORE_MACS.
//
// The estimate and the device list are shown on the TFT, with a live scan
// strip at the bottom (current channel, per-channel activity, BLE activity).
// D6 drives a relay: ON while the estimate exceeds RELAY_THRESHOLD (default
// 0, i.e. one or more devices), OFF once it has stayed at or below it for
// RELAY_OFF_DELAY_MS (default 0 = immediately). D7 gives a PULSE_MS pulse
// every time the estimate changes, up or down.
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

// ---- Pin map: XIAO pad labels, so the same wiring works on C3 and C5 ------
// (the board variant maps D-labels to GPIOs; GPIO numbers in comments).
//                                    C3      C5
const int PIN_SCK     = D8;   // TFT SCK   GPIO8   GPIO8
const int PIN_MOSI    = D10;  // TFT SDA   GPIO10  GPIO10
const int PIN_TFT_CS  = D1;   // TFT CS    GPIO3   GPIO0
const int PIN_TFT_DC  = D2;   // TFT A0    GPIO4   GPIO25
const int PIN_TFT_RST = D3;   // TFT RESET GPIO5   GPIO7
const int PIN_TFT_LED = D4;   // backlight GPIO6   GPIO23
const int PIN_RELAY   = D6;   // relay IN  GPIO21  GPIO11
const int PIN_PULSE   = D7;   // pulse out GPIO20  GPIO12

#if SOC_WIFI_SUPPORT_5G
#define HAS_5G 1
#else
#define HAS_5G 0
#endif

// ---- Tunables -------------------------------------------------------------
// How long an address still counts as "present" after its last frame —
// this is the main "how fast does it drop" knob. Phones advertise BLE many
// times per second, so BLE can be short; connected-but-sleeping phones only
// send Wi-Fi keepalives every few seconds, so Wi-Fi needs a bit longer.
constexpr uint32_t WIFI_SEEN_WINDOW_MS = 15000;
constexpr uint32_t BLE_SEEN_WINDOW_MS  = 8000;
// Channel hopping. Every supported channel is visited each sweep; dwell
// depends on what was heard there within the last ACTIVITY_MEMORY_MS.
constexpr uint32_t IDLE_DWELL_MS     = 110;    // nothing heard: just over one beacon interval (102.4 ms)
constexpr uint32_t AP_DWELL_MS       = 200;    // frames heard (APs) but no counted clients
constexpr uint32_t BUSY_DWELL_MS     = 400;    // counted client devices seen here
constexpr uint32_t ACTIVITY_MEMORY_MS = 30000;
// The Wi-Fi presence window is stretched to at least this many sweeps, so a
// device isn't dropped just because we were busy on other channels.
constexpr uint32_t MIN_SWEEPS_IN_WINDOW = 3;
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
// The scan strip at the bottom has its own small canvas so it can be redrawn
// on every channel hop without pushing the whole screen.
constexpr int SCREEN_W = 128, SCREEN_H = 160, STRIP_H = 22, MAIN_H = SCREEN_H - STRIP_H;
GFXcanvas16 canvas(SCREEN_W, MAIN_H);
GFXcanvas16 strip(SCREEN_W, STRIP_H);

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
  uint32_t windowMs;  // may be stretched at runtime (see MIN_SWEEPS_IN_WINDOW)
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
volatile uint32_t lastAdvertMs = 0;  // for the BLE activity dot

// ---- Channel table (filled at boot with what the driver accepts) ---------
constexpr size_t MAX_CHANNELS = 48;
uint8_t channels[MAX_CHANNELS];
size_t numChannels = 0;
size_t num24 = 0;                       // first num24 entries are 2.4 GHz
volatile size_t curChan = 0;            // index into channels[]
volatile uint32_t chanFrameMs[MAX_CHANNELS];   // last any-frame time (0 = never)
volatile uint32_t chanClientMs[MAX_CHANNELS];  // last counted-client time (0 = never)
uint32_t sweepMs = 0;                   // duration of the last full sweep

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
  rawPackets = rawPackets + 1;
  const uint32_t stamp = millis() | 1;  // never 0, which means "never"
  const size_t ch = curChan;
  chanFrameMs[ch] = stamp;
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
  chanClientMs[ch] = stamp;
  wifiSeen.touch(srcMac, pkt->rx_ctrl.rssi);
}

static void setupSniffer() {
  // Promiscuous mode needs the Wi-Fi driver started. NOTE: in Arduino,
  // WIFI_MODE_NULL == WIFI_OFF and de-inits the driver, so use STA mode
  // and simply never connect.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
#if HAS_5G
  checkErr("set_band_mode", esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO));
#endif
  // Unlock every channel the radio can tune to. We only ever LISTEN
  // (promiscuous mode, never connect or scan actively), so nothing is
  // transmitted on channels that may not be allowed where you are.
  wifi_country_t country = {};
  strcpy(country.cc, "01");
  country.schan = 1;
  country.nchan = 14;
  country.max_tx_power = 20;
  country.policy = WIFI_COUNTRY_POLICY_MANUAL;
#if HAS_5G
  country.wifi_5g_channel_mask = 0x1FFFFFFE;  // bits 1..28 = channels 36..177
#endif
  checkErr("set_country", esp_wifi_set_country(&country));

  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
  };
  checkErr("set_promiscuous_filter", esp_wifi_set_promiscuous_filter(&filter));
  checkErr("set_promiscuous_rx_cb", esp_wifi_set_promiscuous_rx_cb(&onPacket));
  checkErr("set_promiscuous", esp_wifi_set_promiscuous(true));

  // Discover the channel list: keep every candidate the driver accepts.
  auto tryChannel = [](uint8_t ch) {
    if (numChannels < MAX_CHANNELS && esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK)
      channels[numChannels++] = ch;
  };
  for (uint8_t ch = 1; ch <= 14; ch++) tryChannel(ch);
  num24 = numChannels;
#if HAS_5G
  for (uint8_t ch = 36; ch <= 64; ch += 4) tryChannel(ch);
  for (uint8_t ch = 100; ch <= 144; ch += 4) tryChannel(ch);
  for (uint8_t ch = 149; ch <= 177; ch += 4) tryChannel(ch);
#endif
  Serial.printf("scanning %u channels (%u x 2.4GHz, %u x 5GHz):",
                (unsigned)numChannels, (unsigned)num24, (unsigned)(numChannels - num24));
  for (size_t i = 0; i < numChannels; i++) Serial.printf(" %u", channels[i]);
  Serial.println();
  if (numChannels == 0) {  // shouldn't happen; fall back to the classics
    channels[0] = 1; channels[1] = 6; channels[2] = 11;
    numChannels = num24 = 3;
  }
  curChan = 0;
  esp_wifi_set_channel(channels[0], WIFI_SECOND_CHAN_NONE);
}

static bool recent(uint32_t stampMs, uint32_t now) {
  return stampMs != 0 && now - stampMs < ACTIVITY_MEMORY_MS;
}

// How long to stay on channel index i this visit.
static uint32_t dwellFor(size_t i, uint32_t now) {
  if (recent(chanClientMs[i], now)) return BUSY_DWELL_MS;
  if (recent(chanFrameMs[i], now)) return AP_DWELL_MS;
  return IDLE_DWELL_MS;
}

static void hopChannel(uint32_t now) {
  static uint32_t sweepStart = 0;
  size_t next = (curChan + 1) % numChannels;
  if (next == 0) {
    sweepMs = now - sweepStart;
    sweepStart = now;
    // Keep a device for at least MIN_SWEEPS_IN_WINDOW sweeps.
    wifiSeen.windowMs = max(WIFI_SEEN_WINDOW_MS, MIN_SWEEPS_IN_WINDOW * sweepMs);
  }
  curChan = next;
  esp_wifi_set_channel(channels[next], WIFI_SECOND_CHAN_NONE);
}

// ---- BLE scanner ----------------------------------------------------------
class AdvertCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    rawAdverts = rawAdverts + 1;
    lastAdvertMs = millis() | 1;
    if (!dev.haveRSSI() || dev.getRSSI() < BLE_MIN_RSSI) return;
    // Copy the address out of the BLEAddress (getAddress() returns a
    // temporary). Byte order doesn't matter; uniqueness is all we need.
    BLEAddress addr = dev.getAddress();
    uint8_t mac[6];
    memcpy(mac, addr.getNative(), 6);
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
  const size_t rows = (MAIN_H - ROW_Y0) / ROW_H;
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

// Scan strip (bottom STRIP_H px), redrawn on every channel hop:
//   "ch 36 5G  sweep 4.6s    B o"   current channel, last sweep time, BLE dot
//   [|||||||||  |||||||||||||||]   one cell per channel, 2.4 | 5 GHz groups
// Cell colours: green = counted clients recently, blue = other traffic
// (APs) recently, dark grey = quiet. The current channel is a tall white cell.
static void drawStrip(uint32_t now) {
  GFXcanvas16 &s = strip;
  s.fillScreen(ST77XX_BLACK);
  s.drawFastHLine(0, 0, SCREEN_W, 0x7BEF);

  const uint8_t ch = channels[curChan];
  s.setTextSize(1);
  s.setCursor(2, 3);
  s.setTextColor(ST77XX_CYAN);
  s.printf("ch%-3u%s", ch, curChan >= num24 ? "5G" : "2G");
  s.setTextColor(0x7BEF);
  s.setCursor(50, 3);
  if (sweepMs) s.printf("%.1fs", sweepMs / 1000.0f);
  else s.print("...");
  // BLE: dot lights for 300 ms after any advert.
  s.setTextColor(ST77XX_MAGENTA);
  s.setCursor(104, 3);
  s.print("BLE");
  const bool bleLive = lastAdvertMs && now - lastAdvertMs < 300;
  if (bleLive) s.fillCircle(124, 6, 3, ST77XX_MAGENTA);
  else s.drawCircle(124, 6, 3, 0x4208);

  // Channel cells across the full width, with a 3 px gap between bands.
  const bool twoBands = numChannels > num24;
  const int gap = twoBands ? 3 : 0;
  const int usable = SCREEN_W - 2 - gap;
  for (size_t i = 0; i < numChannels; i++) {
    int x0 = 1 + (int)(i * usable / numChannels) + (i >= num24 ? gap : 0);
    int x1 = 1 + (int)((i + 1) * usable / numChannels) + (i >= num24 ? gap : 0);
    int w = max(1, x1 - x0 - 1);
    uint16_t col = recent(chanClientMs[i], now) ? ST77XX_GREEN
                 : recent(chanFrameMs[i], now)  ? 0x341F   // blue
                                                : 0x2104;  // dark grey
    if (i == curChan) s.fillRect(x0, 12, w, 10, ST77XX_WHITE);
    else s.fillRect(x0, 16, w, 5, col);
  }

  tft.drawRGBBitmap(0, MAIN_H, s.getBuffer(), s.width(), s.height());
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

  static uint32_t dwell = IDLE_DWELL_MS;
  if (now - lastHop >= dwell) {
    lastHop = now;
    hopChannel(now);
    dwell = dwellFor(curChan, now);
    drawStrip(now);
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
    Serial.printf("estimate: %u (wifi %u, ble %u)  relay: %s  raw pkts: %lu  raw adverts: %lu"
                  "  ch %u  sweep %lums  wifi window %lums\n",
                  estimate, wifi, ble, relayOn ? "ON" : "off",
                  (unsigned long)rawPackets, (unsigned long)rawAdverts,
                  channels[curChan], (unsigned long)sweepMs, (unsigned long)wifiSeen.windowMs);
  }

  if (DEBUG_LIST && now - lastList >= DEBUG_LIST_MS) {
    lastList = now;
    Serial.println("-- tracked --");
    wifiSeen.printActive("wifi");
    bleSeen.printActive("ble ");
  }
}
