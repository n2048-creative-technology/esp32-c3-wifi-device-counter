// XIAO ESP32-C3 + 128x160 ST7735 SPI TFT — ambient Wi-Fi device counter
// with a relay output.
//
// Passively sniffs 802.11 management/data frames (promiscuous mode, no
// need to join any network) and estimates how many distinct devices are
// nearby by counting unique source MAC addresses seen in a rolling time
// window, hopping across channels 1/6/11. This is the same technique used
// by "Wi-Fi people counters" — it is an ESTIMATE, not an exact count:
//   - iOS/Android MAC randomization in probe requests means one phone can
//     look like several different MACs over time (tends to OVER-count).
//   - Devices that are idle/asleep and not probing won't be seen at all
//     (tends to UNDER-count).
//   - Range depends on antenna/placement, typically tens of metres.
//
// The estimated count is shown on the TFT. D6 (GPIO21) drives a relay:
// ON when the estimate is greater than one device, OFF otherwise (e.g.
// "someone + a guest" vs "just me / empty room").
//
// Based on n2048-creative-technology/esp32-c3-tft-sdcard-looper- (same
// board + screen). The SD card and its SPI lines are not used here.

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ---- Pin map (XIAO label -> GPIO), matching the looper project ----------
constexpr int PIN_SCK     = 8;   // D8  -> TFT SCK
constexpr int PIN_MOSI    = 10;  // D10 -> TFT SDA (MOSI)
constexpr int PIN_TFT_CS  = 3;   // D1  -> CS
constexpr int PIN_TFT_DC  = 4;   // D2  -> A0 (data/command)
constexpr int PIN_TFT_RST = 5;   // D3  -> RESET
constexpr int PIN_TFT_LED = 6;   // D4  -> LED (backlight, PWM)
constexpr int PIN_RELAY   = 21;  // D6  -> relay module IN (free pin on the looper board)
// D7 (GPIO20) stays free for a future second relay / status input.

// ---- Tunables -------------------------------------------------------------
constexpr uint32_t SEEN_WINDOW_MS   = 120000;  // how long a MAC counts as "present" after last seen
constexpr uint32_t CHANNEL_DWELL_MS = 300;     // time spent per Wi-Fi channel before hopping
constexpr uint8_t  CHANNELS[]       = {1, 6, 11};  // non-overlapping 2.4GHz channels
constexpr size_t   MAX_TRACKED_MACS = 128;     // ring buffer size for seen devices
constexpr uint8_t  RELAY_THRESHOLD  = 1;        // relay turns ON when count > this
constexpr bool     RELAY_ACTIVE_HIGH = true;    // flip to false for active-low relay modules
constexpr uint32_t DISPLAY_REFRESH_MS = 500;    // how often the screen number updates
constexpr int8_t   MIN_RSSI          = -90;     // ignore frames weaker than this (noise floor)

constexpr uint32_t TFT_SPI_HZ = 27000000;

Adafruit_ST7735 tft(&SPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

// ---- MAC tracking table (fixed-size, no heap churn in the sniffer ISR) --
struct SeenMac {
  uint8_t mac[6] = {0};
  uint32_t lastSeenMs = 0;
  bool used = false;
};
SeenMac tracked[MAX_TRACKED_MACS];
volatile size_t trackedCount = 0;  // approximate, read by UI loop only

static bool macEquals(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

// Called from the Wi-Fi driver's own task — keep this fast and allocation-free.
static void IRAM_ATTR onPacket(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  auto *pkt = (wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.rssi < MIN_RSSI) return;

  // Source MAC sits at the same offset (byte 10) for both the management
  // and data frame formats we care about (addr2 / transmitter address).
  const uint8_t *payload = pkt->payload;
  if (pkt->rx_ctrl.sig_len < 16) return;
  const uint8_t *srcMac = payload + 10;

  // Ignore broadcast/multicast and all-zero addresses (malformed/noise).
  if (srcMac[0] & 0x01) return;
  bool allZero = true;
  for (int i = 0; i < 6; i++) if (srcMac[i] != 0) { allZero = false; break; }
  if (allZero) return;

  uint32_t now = millis();
  int freeSlot = -1;
  for (size_t i = 0; i < MAX_TRACKED_MACS; i++) {
    if (tracked[i].used && macEquals(tracked[i].mac, srcMac)) {
      tracked[i].lastSeenMs = now;
      return;
    }
    if (!tracked[i].used && freeSlot < 0) freeSlot = (int)i;
  }
  if (freeSlot < 0) return;  // table full; drop (undercount under heavy load)
  memcpy(tracked[freeSlot].mac, srcMac, 6);
  tracked[freeSlot].lastSeenMs = now;
  tracked[freeSlot].used = true;
}

// Sweeps the table, expiring stale entries, and returns the live count.
// Only ever called from the main loop (not the sniffer callback).
static uint16_t countActiveDevices() {
  uint32_t now = millis();
  uint16_t count = 0;
  for (size_t i = 0; i < MAX_TRACKED_MACS; i++) {
    if (!tracked[i].used) continue;
    if (now - tracked[i].lastSeenMs > SEEN_WINDOW_MS) {
      tracked[i].used = false;
      continue;
    }
    count++;
  }
  return count;
}

static void setupSniffer() {
  WiFi.mode(WIFI_MODE_NULL);  // promiscuous mode needs Wi-Fi started but not connected
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&onPacket);
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
  };
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_channel(CHANNELS[0], WIFI_SECOND_CHAN_NONE);
}

static void hopChannel() {
  static size_t idx = 0;
  idx = (idx + 1) % (sizeof(CHANNELS) / sizeof(CHANNELS[0]));
  esp_wifi_set_channel(CHANNELS[idx], WIFI_SECOND_CHAN_NONE);
}

// ---- Display --------------------------------------------------------------
static void drawCount(uint16_t count, bool relayOn) {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.print("devices nearby");

  // Big centred number.
  tft.setTextSize(6);
  char buf[4];
  snprintf(buf, sizeof(buf), "%u", min(count, (uint16_t)999));
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((tft.width() - w) / 2, (tft.height() - h) / 2);
  tft.setTextColor(count > RELAY_THRESHOLD ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.print(buf);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(4, tft.height() - 12);
  tft.print(relayOn ? "relay: ON" : "relay: off");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_HIGH ? LOW : HIGH);  // start OFF

  pinMode(PIN_TFT_LED, OUTPUT);
  analogWrite(PIN_TFT_LED, 200);

  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_TFT_CS);
  SPI.setFrequency(TFT_SPI_HZ);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);

  setupSniffer();
  Serial.println("Wi-Fi device counter started (promiscuous sniffer).");
}

void loop() {
  static uint32_t lastHop = 0, lastDraw = 0;
  uint32_t now = millis();

  if (now - lastHop >= CHANNEL_DWELL_MS) {
    lastHop = now;
    hopChannel();
  }

  if (now - lastDraw >= DISPLAY_REFRESH_MS) {
    lastDraw = now;
    uint16_t count = countActiveDevices();
    bool relayOn = count > RELAY_THRESHOLD;
    // XOR: active-high board -> HIGH means ON; active-low board -> LOW means ON.
    digitalWrite(PIN_RELAY, relayOn == RELAY_ACTIVE_HIGH ? HIGH : LOW);
    drawCount(count, relayOn);
    Serial.printf("estimated devices: %u  relay: %s\n", count, relayOn ? "ON" : "off");
  }
}
