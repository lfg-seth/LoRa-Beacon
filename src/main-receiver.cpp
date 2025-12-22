/**
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262) - RECEIVER
 * - Raw LoRa RX/TX (no LoRaWAN)
 * - Receives TelemetryPacket (binary) + CRC16 (PING)
 * - Logs every packet with useful RF metrics (RSSI/SNR/FreqError/Len)
 * - Each time a valid PING is received, immediately sends an AckPacket (PONG)
 *   containing the receiver-measured RF metrics for that PING.
 *
 * Assumes the beacon uses:
 *   FREQUENCY 910.525 MHz, BW 125 kHz, SF9
 */

#define HELTEC_POWER_BUTTON
#define HELTEC_WIRELESS_STICK
#include <heltec_unofficial.h>

#include <Arduino.h>
#include <math.h>

// -------------------- Radio settings (MUST match beacon) --------------------
#define FREQUENCY 910.525 // MHz
#define BANDWIDTH 125.0   // kHz
#define SPREADING_FACTOR 9

// -------------------- OLED timing --------------------
static const uint32_t OLED_PERIOD_MS = 250;

// -------------------- RX globals --------------------
volatile bool rxFlag = false;
static uint32_t last_oled_ms = 0;

static float rxRSSI = NAN;
static float rxSNR = NAN;
static int32_t rxFreqErrHz = 0x7FFFFFFF;
static uint32_t last_rx_ms = 0;
static uint32_t good_pkts = 0;
static uint32_t bad_pkts = 0;

// -------------------- CRC16 --------------------
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

// -------------------- Telemetry packet (MUST match beacon) --------------------
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

// -------------------- Simple Signal Quality (OLED) --------------------
enum SigQuality : uint8_t
{
  SIG_POOR = 0,
  SIG_GOOD = 1,
  SIG_EXCELLENT = 2
};

static SigQuality signalQuality(float rssi_dBm, float snr_dB)
{
  if (isnan(rssi_dBm) || isnan(snr_dB))
    return SIG_POOR;

  // Same thresholds as beacon OLED (tune later)
  if (snr_dB >= 4.0f && rssi_dBm >= -110.0f)
    return SIG_EXCELLENT;
  if (snr_dB >= -2.0f && rssi_dBm >= -120.0f)
    return SIG_GOOD;
  return SIG_POOR;
}

static const char *sigQualityLabel(SigQuality q)
{
  switch (q)
  {
  case SIG_EXCELLENT:
    return "EXCL";
  case SIG_GOOD:
    return "GOOD";
  default:
    return "POOR";
  }
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

static float pktRssiFromTelemetry(const TelemetryPacket &p)
{
  if (p.rssi_dBm_x1 == (int16_t)0x7FFF)
    return NAN;
  return (float)p.rssi_dBm_x1;
}

static float pktSnrFromTelemetry(const TelemetryPacket &p)
{
  if (p.snr_dB_x10 == (int16_t)0x7FFF)
    return NAN;
  return (float)p.snr_dB_x10 / 10.0f;
}

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

// -------------------- ACK packet (PONG) --------------------
struct __attribute__((packed)) AckPacket
{
  uint32_t magic;  // 'ACK1' = 0x314B4341 (little-endian)
  uint8_t version; // 1
  uint8_t flags;   // reserved
  uint16_t seq;    // echoed TelemetryPacket.seq

  uint32_t rx_uptime_s; // receiver uptime when ACK built

  int16_t ping_rssi_dBm_x1; // RSSI seen on PING
  int16_t ping_snr_dB_x10;  // SNR*10 seen on PING
  int32_t ping_freqerr_Hz;  // freq error on PING (if available) else 0x7FFFFFFF
  uint16_t ping_len;        // PING length in bytes

  uint16_t crc; // CRC16-CCITT of all prior bytes
};

static const uint32_t ACK_MAGIC = 0x314B4341; // 'ACK1'

static void buildAck(AckPacket &ack, uint16_t seq, uint16_t ping_len)
{
  memset(&ack, 0, sizeof(ack));
  ack.magic = ACK_MAGIC;
  ack.version = 1;
  ack.seq = seq;
  ack.rx_uptime_s = millis() / 1000;

  // Receiver-measured metrics for the PING
  const double r = isnan(rxRSSI) ? 0.0 : (double)rxRSSI;
  const double s10 = isnan(rxSNR) ? 0.0 : ((double)rxSNR * 10.0);

  double rclamp = r;
  if (rclamp > 32767)
    rclamp = 32767;
  if (rclamp < -32768)
    rclamp = -32768;
  ack.ping_rssi_dBm_x1 = (int16_t)llround(rclamp);

  double s10clamp = s10;
  if (s10clamp > 32767)
    s10clamp = 32767;
  if (s10clamp < -32768)
    s10clamp = -32768;
  ack.ping_snr_dB_x10 = (int16_t)llround(s10clamp);

  ack.ping_freqerr_Hz = rxFreqErrHz;
  ack.ping_len = ping_len;

  ack.crc = 0;
  ack.crc = crc16_ccitt((const uint8_t *)&ack, sizeof(ack) - sizeof(ack.crc));
}

// Store last good packet for OLED
static TelemetryPacket lastPkt;
static bool hasPkt = false;

// -------------------- ISR --------------------
void rx() { rxFlag = true; }

// -------------------- OLED helpers (Beacon-style, last line at y=48) --------------------
static void drawOLED()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  uint32_t now = millis();

  if (!hasPkt)
  {
    display.drawString(0, 0, "Lat: -----.----- " + String(good_pkts) + "/" + String(bad_pkts));
    display.drawString(0, 12, "Lon: -----.----- PK:--");
    display.drawString(0, 24, "Alt:-- HDOP:-- S:--");
    display.drawString(0, 36, "PING --");
    display.drawString(0, 48, "PONG --");
    display.display();
    return;
  }

  // Decode scaled values from lastPkt (beacon GPS data)
  const double lat = (double)lastPkt.lat_e7 / 1e7;
  const double lon = (double)lastPkt.lon_e7 / 1e7;
  const double alt_m = (lastPkt.alt_dm == (int16_t)0x7FFF) ? NAN : ((double)lastPkt.alt_dm / 10.0);
  const double hdop = (lastPkt.hdop_c == 0xFFFF) ? NAN : ((double)lastPkt.hdop_c / 100.0);

  const bool hasFix = (lastPkt.flags & (1u << 0)) != 0;
  const bool avgUsed = (lastPkt.flags & (1u << 1)) != 0;

  // Line 0: Lat + good/bad (in place of SD)
  String latStr = hasFix ? String(lat, 5) : String("-----.-----");
  display.drawString(0, 0, "Lat: " + latStr + " " + String(good_pkts) + "/" + String(bad_pkts));

  // Line 12: Lon + packet age
  String lonStr = hasFix ? String(lon, 5) : String("-----.-----");
  uint32_t age_ms = (last_rx_ms > 0) ? (now - last_rx_ms) : 0;
  display.drawString(0, 12, "Lon: " + lonStr + " PK:" + String((unsigned long)age_ms));

  // Line 24: Alt/HDOP/Sats (+AVG)
  String altStr = isnan(alt_m) ? String("--") : (String(alt_m, 0) + "m");
  String hdopStr = isnan(hdop) ? String("--") : String(hdop, 2);
  String satsStr = (lastPkt.sats > 0) ? String(lastPkt.sats) : String("--");

  String l24 = "Alt:" + altStr + " HDOP:" + hdopStr + " S:" + satsStr;
  if (avgUsed)
    l24 += " AVG";
  display.drawString(0, 24, l24);

  // Line 36: PING quality (receiver measured RSSI/SNR for incoming PING)
  SigQuality pingQ = signalQuality(rxRSSI, rxSNR);
  display.drawString(
      0, 36,
      "PING " + String(sigQualityLabel(pingQ)) +
          " R:" + (isnan(rxRSSI) ? String("--") : String(rxRSSI, 0)) +
          " S:" + (isnan(rxSNR) ? String("--") : String(rxSNR, 1)));

  // Line 48: PONG quality proxy = what beacon reports it last RX'd (usually your previous PONG)
  float bRssi = pktRssiFromTelemetry(lastPkt);
  float bSnr = pktSnrFromTelemetry(lastPkt);
  SigQuality pongQ = signalQuality(bRssi, bSnr);

  display.drawString(
      0, 48,
      "PONG " + String(sigQualityLabel(pongQ)) +
          " R:" + (isnan(bRssi) ? String("--") : String(bRssi, 0)) +
          " S:" + (isnan(bSnr) ? String("--") : String(bSnr, 1)));

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

static int32_t tryGetFreqErrHz()
{
  // If RadioLib exposes it, use it; otherwise keep sentinel.
  int32_t fe = 0x7FFFFFFF;
#if defined(RADIOLIB_VERSION)
  fe = (int32_t)radio.getFrequencyError();
#endif
  return fe;
}

static void logPacketToSerial(const TelemetryPacket &p, uint16_t got_len)
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

  char feBuf[24];
  if (rxFreqErrHz == 0x7FFFFFFF)
    snprintf(feBuf, sizeof(feBuf), "INV");
  else
    snprintf(feBuf, sizeof(feBuf), "%ld", (long)rxFreqErrHz);

  Serial.printf(
      "PING v%u seq=%u up=%lus flags{fix=%u avg=%u chg=%u} gps{%s sats=%u hdop=%s alt=%s lat=%s lon=%s} "
      "pwr{I=%dmA V=%.3f} mAh{net=%.3f in=%.3f out=%.3f} "
      "rf{len=%u R=%.1f S=%.1f FE=%sHz}\n",
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
      (unsigned)got_len, rxRSSI, rxSNR, feBuf);
}

static void sendAck(uint16_t seq, uint16_t ping_len)
{
  AckPacket ack;
  buildAck(ack, seq, ping_len);

  radio.clearDio1Action();

  uint32_t t0 = millis();
  RADIOLIB(radio.transmit((uint8_t *)&ack, sizeof(ack)));
  uint32_t dt = millis() - t0;

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    Serial.printf("PONG TX fail: %d\n", _radiolib_status);
  }
  else
  {
    Serial.printf("PONG tx %u bytes seq=%u (air=%lums)\n",
                  (unsigned)sizeof(ack), (unsigned)seq, (unsigned long)dt);
  }

  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

static void handleRX()
{
  if (!rxFlag)
    return;
  rxFlag = false;

  uint8_t buf[sizeof(TelemetryPacket)] = {0};
  int16_t len = radio.getPacketLength(true);

  size_t toRead = (len > 0 && (size_t)len <= sizeof(buf)) ? (size_t)len : sizeof(buf);
  radio.readData(buf, toRead);

  rxRSSI = radio.getRSSI();
  rxSNR = radio.getSNR();
  rxFreqErrHz = tryGetFreqErrHz();
  last_rx_ms = millis();

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    bad_pkts++;
    Serial.printf("RX read fail: %d RSSI=%.1f SNR=%.1f len=%d\n", _radiolib_status, rxRSSI, rxSNR, (int)len);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    return;
  }

  if (len != (int16_t)sizeof(TelemetryPacket))
  {
    bad_pkts++;
    Serial.printf("RX len mismatch: got=%d expected=%u RSSI=%.1f SNR=%.1f\n",
                  (int)len, (unsigned)sizeof(TelemetryPacket), rxRSSI, rxSNR);
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

  logPacketToSerial(pkt, (uint16_t)len);

  // Immediately reply with PONG containing the RF metrics we just measured.
  sendAck(pkt.seq, (uint16_t)len);
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
  Serial.printf("AckPacket size = %u bytes\n", (unsigned)sizeof(AckPacket));
}

void loop()
{
  heltec_loop();

  handleRX();
  oledTick();
}
