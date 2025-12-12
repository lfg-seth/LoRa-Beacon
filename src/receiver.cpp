/**
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262) - RECEIVER
 * - Raw LoRa RX (no LoRaWAN)
 * - Receives TelemetryPacket (binary, 40 bytes) + CRC16
 * - Logs every valid packet to Serial
 * - OLED shows a compact summary (GPS quality, sats/hdop, lat/lon, I/V, uptime)
 *
 * Assumes the transmitter uses:
 *   FREQUENCY 910.525 MHz, BW 125 kHz, SF9, PWR 10 dBm
 * and the exact TelemetryPacket definition below.
 */

#define HELTEC_POWER_BUTTON
#define HELTEC_WIRELESS_STICK
#include <heltec_unofficial.h>

#include <Arduino.h>

// -------------------- Radio settings (MUST match TX) --------------------
#define FREQUENCY 910.525 // MHz
#define BANDWIDTH 125.0   // kHz
#define SPREADING_FACTOR 9

// -------------------- OLED timing --------------------
static const uint32_t OLED_PERIOD_MS = 250;

// -------------------- RX globals --------------------
volatile bool rxFlag = false;
uint32_t last_oled_ms = 0;

static float rxRSSI = NAN;
static float rxSNR = NAN;
static uint32_t last_rx_ms = 0;
static uint32_t good_pkts = 0;
static uint32_t bad_pkts = 0;

// -------------------- Telemetry packet (MUST match TX) --------------------
enum GpsQuality : uint8_t
{
  GPS_NOFIX = 0,
  GPS_POOR = 1,
  GPS_NORMAL = 2,
  GPS_EXCELLENT = 3
};

static const char *gpsQualityLabel(uint8_t q)
{
  switch ((GpsQuality)q)
  {
  case GPS_EXCELLENT:
    return "EXCELLENT";
  case GPS_NORMAL:
    return "NORMAL";
  case GPS_POOR:
    return "POOR";
  case GPS_NOFIX:
  default:
    return "NOFIX";
  }
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++)
    {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

struct __attribute__((packed)) TelemetryPacket
{
  uint32_t magic;    // 'TLMS' = 0x534D4C54 (little-endian)
  uint8_t version;   // 1
  uint8_t flags;     // bit0: has_fix, bit1: avg_used, bit2: charging
  uint16_t seq;      // txCounter mod 65536
  uint32_t uptime_s; // millis()/1000 (TX side)

  int32_t lat_e7;      // deg * 1e7
  int32_t lon_e7;      // deg * 1e7
  int16_t alt_dm;      // meters * 10
  uint16_t hdop_c;     // hdop * 100
  uint8_t sats;        // satellites used
  uint8_t gps_quality; // enum above

  int16_t current_mA; // signed
  uint16_t v_mV;      // loadV * 1000

  int32_t net_mAh_x1000;  // net mAh * 1000
  uint32_t in_mAh_x1000;  // in mAh * 1000
  uint32_t out_mAh_x1000; // out mAh * 1000

  int16_t rssi_dBm_x1; // last RX RSSI (if recent) else 0x7FFF
  int16_t snr_dB_x10;  // last RX SNR * 10 (if recent) else 0x7FFF

  uint16_t crc; // CRC16-CCITT of all prior bytes
};

static const uint32_t TLMS_MAGIC = 0x534D4C54; // 'TLMS'

static bool decodeTelemetry(const uint8_t *buf, size_t len, TelemetryPacket &out)
{
  if (len != sizeof(TelemetryPacket))
    return false;
  memcpy(&out, buf, sizeof(out));
  if (out.magic != TLMS_MAGIC)
    return false;

  uint16_t got = out.crc;
  TelemetryPacket tmp = out;
  tmp.crc = 0;
  uint16_t calc = crc16_ccitt((const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc));
  return got == calc;
}

// Store last good packet for OLED
static TelemetryPacket lastPkt;
static bool hasPkt = false;

// -------------------- ISR --------------------
void rx() { rxFlag = true; }

// -------------------- OLED helpers --------------------
static void drawOLED()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  display.drawString(0, 0, "LoRa TLM RX");

  // Top-right: link stats
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  String link = String(good_pkts) + "/" + String(bad_pkts);
  display.drawString(128, 0, link);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  if (!hasPkt)
  {
    display.drawString(0, 14, "Waiting for packets...");
    display.display();
    return;
  }

  // Decode scaled values
  const double lat = (double)lastPkt.lat_e7 / 1e7;
  const double lon = (double)lastPkt.lon_e7 / 1e7;
  const double alt_m = (lastPkt.alt_dm == (int16_t)0x7FFF) ? NAN : ((double)lastPkt.alt_dm / 10.0);
  const double hdop = (lastPkt.hdop_c == 0xFFFF) ? NAN : ((double)lastPkt.hdop_c / 100.0);
  const double volts = (double)lastPkt.v_mV / 1000.0;
  const int cur_mA = (int)lastPkt.current_mA;

  const bool hasFix = (lastPkt.flags & (1u << 0)) != 0;
  const bool avgUsed = (lastPkt.flags & (1u << 1)) != 0;

  // Line 2: quality + sats + hdop
  String l2 = String(gpsQualityLabel(lastPkt.gps_quality)).substring(0, 4);
  l2 += " S:" + String(lastPkt.sats);
  l2 += " H:" + (isnan(hdop) ? String("--") : String(hdop, 2));
  if (avgUsed)
    l2 += " AVG";
  display.drawString(0, 12, l2);

  // Line 3: Lat
  display.drawString(0, 24, "Lat:" + (hasFix ? String(lat, 5) : String("-----.-----")));

  // Line 4: Lon
  display.drawString(0, 36, "Lon:" + (hasFix ? String(lon, 5) : String("-----.-----")));

  // Line 5: Alt + I/V
  String altStr = isnan(alt_m) ? String("--") : String(alt_m, 0);
  String l5 = "Alt:" + altStr + " I:" + String(cur_mA) + "mA";
  display.drawString(0, 48, l5);

  // Line 6: V + seq + RSSI
  String l6 = "V:" + String(volts, 2) + " Seq:" + String(lastPkt.seq);
  if (!isnan(rxRSSI))
    l6 += " R:" + String(rxRSSI, 0);
  display.drawString(0, 56, l6);

  display.display();
}

static void oledTick()
{
  uint32_t now = millis();
  if (now - last_oled_ms < OLED_PERIOD_MS)
    return;
  last_oled_ms = now;
  drawOLED();
}

// -------------------- Radio helpers --------------------
static void radioInit()
{
  Serial.println("Radio init...");
  RADIOLIB_OR_HALT(radio.begin());

  radio.setDio1Action(rx);

  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));

  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

static void logPacketToSerial(const TelemetryPacket &p)
{
  const double lat = (double)p.lat_e7 / 1e7;
  const double lon = (double)p.lon_e7 / 1e7;
  const double alt_m = (p.alt_dm == (int16_t)0x7FFF) ? NAN : ((double)p.alt_dm / 10.0);
  const double hdop = (p.hdop_c == 0xFFFF) ? NAN : ((double)p.hdop_c / 100.0);

  const double volts = (double)p.v_mV / 1000.0;
  const int cur_mA = (int)p.current_mA;

  const double net_mAh = (double)p.net_mAh_x1000 / 1000.0;
  const double in_mAh = (double)p.in_mAh_x1000 / 1000.0;
  const double out_mAh = (double)p.out_mAh_x1000 / 1000.0;

  const bool hasFix = (p.flags & (1u << 0)) != 0;
  const bool avgUsed = (p.flags & (1u << 1)) != 0;
  const bool charging = (p.flags & (1u << 2)) != 0;

  Serial.printf(
      "TLM v%u seq=%u up=%lus flags{fix=%u avg=%u chg=%u} gps{%s sats=%u hdop=%s alt=%s lat=%s lon=%s} "
      "pwr{I=%dmA V=%.3f} mAh{net=%.3f in=%.3f out=%.3f} "
      "rx{R=%.1f S=%.1f} tx_rx{R=%s S=%s}\n",
      (unsigned)p.version,
      (unsigned)p.seq,
      (unsigned long)p.uptime_s,
      hasFix ? 1 : 0, avgUsed ? 1 : 0, charging ? 1 : 0,
      gpsQualityLabel(p.gps_quality),
      (unsigned)p.sats,
      isnan(hdop) ? "INV" : String(hdop, 2).c_str(),
      isnan(alt_m) ? "INV" : String(alt_m, 1).c_str(),
      hasFix ? String(lat, 5).c_str() : "INV",
      hasFix ? String(lon, 5).c_str() : "INV",
      cur_mA, volts,
      net_mAh, in_mAh, out_mAh,
      rxRSSI, rxSNR,
      (p.rssi_dBm_x1 == (int16_t)0x7FFF) ? "INV" : String((int)p.rssi_dBm_x1).c_str(),
      (p.snr_dB_x10 == (int16_t)0x7FFF) ? "INV" : String((double)p.snr_dB_x10 / 10.0, 1).c_str());
}

static void handleRX()
{
  if (!rxFlag)
    return;
  rxFlag = false;

  uint8_t buf[sizeof(TelemetryPacket)] = {0};
  int16_t len = radio.getPacketLength(true);

  // If length doesn't match, drain into buffer up to our size anyway
  if (len != (int16_t)sizeof(TelemetryPacket))
  {
    // Read raw bytes (RadioLib will read up to provided length)
    size_t toRead = (len > 0 && (size_t)len < sizeof(buf)) ? (size_t)len : sizeof(buf);
    radio.readData(buf, toRead);

    rxRSSI = radio.getRSSI();
    rxSNR = radio.getSNR();
    last_rx_ms = millis();

    bad_pkts++;
    Serial.printf("RX len mismatch: got=%d expected=%u RSSI=%.1f SNR=%.1f\n",
                  (int)len, (unsigned)sizeof(TelemetryPacket), rxRSSI, rxSNR);

    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    return;
  }

  // Read full packet
  radio.readData(buf, sizeof(buf));
  rxRSSI = radio.getRSSI();
  rxSNR = radio.getSNR();
  last_rx_ms = millis();

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    bad_pkts++;
    Serial.printf("RX read fail: %d RSSI=%.1f SNR=%.1f\n", _radiolib_status, rxRSSI, rxSNR);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    return;
  }

  TelemetryPacket pkt;
  if (!decodeTelemetry(buf, sizeof(buf), pkt))
  {
    bad_pkts++;
    Serial.printf("RX bad CRC/magic RSSI=%.1f SNR=%.1f\n", rxRSSI, rxSNR);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    return;
  }

  good_pkts++;
  lastPkt = pkt;
  hasPkt = true;

  logPacketToSerial(pkt);

  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

// -------------------- Arduino entrypoints --------------------
void setup()
{
  Serial.begin(115200);

  heltec_setup();

  display.init();
  display.setFont(ArialMT_Plain_10);

  radioInit();

  drawOLED();

  Serial.printf("Expecting TelemetryPacket size = %u bytes\n", (unsigned)sizeof(TelemetryPacket));
}

void loop()
{
  heltec_loop();

  handleRX();
  oledTick();
}
