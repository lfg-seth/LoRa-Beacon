/**
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262) - BEACON
 * - Raw LoRa TX/RX (no LoRaWAN)
 * - GPS (GT-U7) on UART pins 45/46
 * - microSD logging over custom SPI pins (4/5/2 + CS=6)
 * - Broadcast TelemetryPacket over LoRa + CRC16 (PING)
 * - On ACK (PONG), logs:
 *     1) Original ping seq
 *     2) Receiver-measured metrics for ping (beacon TX data)
 *     3) Beacon-measured metrics for ACK (beacon RX data)
 */

#define HELTEC_POWER_BUTTON
#define HELTEC_WIRELESS_STICK
#include <heltec_unofficial.h>

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// -------------------- LoRa settings --------------------
#define PAUSE 1           // seconds (0 = only button)
#define FREQUENCY 910.525 // MHz
#define BANDWIDTH 125.0   // kHz
#define SPREADING_FACTOR 9
#define TRANSMIT_POWER 10 // dBm

// -------------------- GPS (GT-U7) --------------------
static const int GPS_RX_PIN = 45; // ESP32 RX  <- GPS TX
static const int GPS_TX_PIN = 46; // ESP32 TX  -> GPS RX (optional)
static const uint32_t GPS_BAUD = 9600;

static HardwareSerial GPSSerial(1);
static TinyGPSPlus gps;

static uint32_t last_gps_print_ms = 0;
static const uint32_t GPS_PRINT_PERIOD_MS = 1000;

static const uint32_t GPS_FIX_STALE_MS = 5000;

static const uint32_t GPS_SATS_NORMAL = 7;
static const uint32_t GPS_SATS_EXCELLENT = 10;
static const float GPS_HDOP_NORMAL = 3.0f;
static const float GPS_HDOP_EXCELLENT = 1.5f;

static const uint32_t GPS_AVG_WINDOW_MS = 120000; // 2 minutes
static const uint32_t GPS_AVG_MIN_SAMPLES = 20;

// Cached latest raw values
static bool gps_has_fix = false;
static float gps_lat = NAN, gps_lon = NAN, gps_alt_m = NAN, gps_hdop = NAN;
static uint32_t gps_sats = 0;
static uint32_t gps_last_fix_ms = 0;

// Averaged values
static bool gps_has_avg = false;
static float gps_lat_avg = NAN, gps_lon_avg = NAN, gps_alt_avg_m = NAN, gps_hdop_avg = NAN;
static uint32_t gps_avg_samples = 0;
static uint32_t gps_last_avg_ms = 0;

// Averaging accumulators (weighted by HDOP)
static double gps_wsum = 0.0;
static double gps_lat_wsum = 0.0;
static double gps_lon_wsum = 0.0;
static double gps_alt_wsum = 0.0;
static double gps_hdop_wsum = 0.0;
static uint32_t gps_accum_samples = 0;
static uint32_t gps_window_start_ms = 0;

// -------------------- microSD (custom SPI bus) --------------------
static const int SD_CS = 6;
static const int SD_SCK = 4;
static const int SD_MISO = 2;
static const int SD_MOSI = 5;

static SPIClass SDSPI(FSPI); // separate SPI peripheral
static bool sd_ok = false;
static const char *SD_PATH = "/beacon.csv";

// -------------------- OLED timing --------------------
static const uint32_t OLED_PERIOD_MS = 250;

// -------------------- Globals --------------------
volatile bool rxFlag = false;

long txCounter = 0;
uint32_t last_tx_ms = 0;
uint32_t minimum_pause_ms = 0;
uint32_t tx_time_ms = 0;

// Latest RX stats at BEACON (for whatever it last received)
static float lastRSSI = NAN;
static float lastSNR = NAN;
static uint32_t last_rx_ms = 0;

// OLED tick
static uint32_t last_oled_ms = 0;

// Track last TX packet so we can correlate ACKs
static uint16_t last_tx_seq = 0;
static uint32_t last_tx_start_ms = 0;

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

// -------------------- Telemetry packet --------------------
enum GpsQuality : uint8_t
{
  GPS_NOFIX = 0,
  GPS_POOR = 1,
  GPS_NORMAL = 2,
  GPS_EXCELLENT = 3
};

struct __attribute__((packed)) TelemetryPacket
{
  uint32_t magic;    // 'TLMS' = 0x534D4C54 (little-endian)
  uint8_t version;   // 1
  uint8_t flags;     // bit0: has_fix, bit1: avg_used, bit2: charging (unused now)
  uint16_t seq;      // txCounter mod 65536
  uint32_t uptime_s; // millis()/1000

  int32_t lat_e7;      // deg * 1e7
  int32_t lon_e7;      // deg * 1e7
  int16_t alt_dm;      // meters * 10
  uint16_t hdop_c;     // hdop * 100
  uint8_t sats;        // satellites used
  uint8_t gps_quality; // enum above

  // INA219 removed -> these fields are kept for compatibility and set to 0 / sentinel
  int16_t current_mA; // signed (0)
  uint16_t v_mV;      // (0)

  int32_t net_mAh_x1000;  // (0)
  uint32_t in_mAh_x1000;  // (0)
  uint32_t out_mAh_x1000; // (0)

  int16_t rssi_dBm_x1; // last RX RSSI (if recent) else 0x7FFF
  int16_t snr_dB_x10;  // last RX SNR * 10 (if recent) else 0x7FFF

  uint16_t crc; // CRC16-CCITT of all prior bytes
};

static const uint32_t TLMS_MAGIC = 0x534D4C54; // 'TLMS'

// -------------------- ACK packet (PONG) --------------------
struct __attribute__((packed)) AckPacket
{
  uint32_t magic;  // 'ACK1' = 0x314B4341 (little-endian)
  uint8_t version; // 1
  uint8_t flags;   // reserved
  uint16_t seq;    // echoed TelemetryPacket.seq

  uint32_t rx_uptime_s; // receiver uptime when ACK built

  int16_t ping_rssi_dBm_x1; // RSSI seen by receiver on PING
  int16_t ping_snr_dB_x10;  // SNR*10 seen by receiver on PING
  int32_t ping_freqerr_Hz;  // frequency error seen by receiver on PING (if available), else 0x7FFFFFFF
  uint16_t ping_len;        // length of received PING in bytes (expected 40)

  uint16_t crc; // CRC16-CCITT of all prior bytes
};

static const uint32_t ACK_MAGIC = 0x314B4341; // 'ACK1'

static bool decodeAck(const uint8_t *buf, size_t len, AckPacket &out)
{
  if (len != sizeof(AckPacket))
    return false;
  memcpy(&out, buf, sizeof(out));
  if (out.magic != ACK_MAGIC)
    return false;

  uint16_t got = out.crc;
  AckPacket tmp = out;
  tmp.crc = 0;
  uint16_t calc = crc16_ccitt((const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc));
  return got == calc;
}

// -------------------- ISR --------------------
void rx() { rxFlag = true; }

// -------------------- SD helpers --------------------
static void sdAppendLine(const String &line)
{
  if (!sd_ok)
    return;

  File f = SD.open(SD_PATH, FILE_APPEND);
  if (!f)
  {
    sd_ok = false; // mark failed so OLED shows it
    return;
  }
  f.println(line);
  f.close();
}

static void sdInit()
{
  // Bring up a separate SPI bus for SD
  SDSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SDSPI))
  {
    Serial.println("SD init failed.");
    sd_ok = false;
    return;
  }

  sd_ok = true;
  Serial.println("SD init OK.");

  // If file doesn't exist, create with header
  if (!SD.exists(SD_PATH))
  {
    File f = SD.open(SD_PATH, FILE_WRITE);
    if (f)
    {
      f.println("ms,type,msg");
      f.close();
    }
  }
}

// -------------------- GPS helpers --------------------
static const char *gpsQualityLabel(GpsQuality q)
{
  switch (q)
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

static GpsQuality gpsQualityNow()
{
  if (!gps_has_fix)
    return GPS_NOFIX;
  if ((millis() - gps_last_fix_ms) > GPS_FIX_STALE_MS)
    return GPS_NOFIX;

  if (isnan(gps_hdop) || gps_sats == 0)
    return GPS_POOR;

  if (gps_sats >= GPS_SATS_EXCELLENT && gps_hdop <= GPS_HDOP_EXCELLENT)
    return GPS_EXCELLENT;
  if (gps_sats >= GPS_SATS_NORMAL && gps_hdop <= GPS_HDOP_NORMAL)
    return GPS_NORMAL;
  return GPS_POOR;
}

static void gpsInit()
{
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("GPS init: UART1 %lu baud (RX=%d TX=%d)\n",
                (unsigned long)GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
  gps_window_start_ms = millis();
}

static void gpsResetAverager()
{
  gps_wsum = 0.0;
  gps_lat_wsum = 0.0;
  gps_lon_wsum = 0.0;
  gps_alt_wsum = 0.0;
  gps_hdop_wsum = 0.0;
  gps_accum_samples = 0;
  gps_window_start_ms = millis();
}

static void gpsMaybeFinalizeAverage()
{
  uint32_t now = millis();
  if ((now - gps_window_start_ms) < GPS_AVG_WINDOW_MS)
    return;

  if (gps_accum_samples >= GPS_AVG_MIN_SAMPLES && gps_wsum > 0.0)
  {
    gps_lat_avg = (float)(gps_lat_wsum / gps_wsum);
    gps_lon_avg = (float)(gps_lon_wsum / gps_wsum);
    gps_alt_avg_m = (float)(gps_alt_wsum / gps_wsum);
    gps_hdop_avg = (float)(gps_hdop_wsum / gps_wsum);
    gps_avg_samples = gps_accum_samples;
    gps_last_avg_ms = now;
    gps_has_avg = true;
  }

  gpsResetAverager();
}

static void gpsPoll()
{
  while (GPSSerial.available() > 0)
    gps.encode((char)GPSSerial.read());

  if (gps.location.isValid())
  {
    gps_lat = (float)gps.location.lat();
    gps_lon = (float)gps.location.lng();
    gps_has_fix = true;
    gps_last_fix_ms = millis();
  }
  if (gps.altitude.isValid())
    gps_alt_m = (float)gps.altitude.meters();
  if (gps.hdop.isValid())
    gps_hdop = (float)gps.hdop.hdop();
  if (gps.satellites.isValid())
    gps_sats = (uint32_t)gps.satellites.value();

  GpsQuality q = gpsQualityNow();
  if ((q == GPS_NORMAL || q == GPS_EXCELLENT) && !isnan(gps_hdop) && gps_hdop > 0.0f)
  {
    const double hd = (double)max(0.6f, gps_hdop);
    const double w = 1.0 / (hd * hd);

    gps_wsum += w;
    gps_lat_wsum += (double)gps_lat * w;
    gps_lon_wsum += (double)gps_lon * w;
    if (!isnan(gps_alt_m))
      gps_alt_wsum += (double)gps_alt_m * w;
    if (!isnan(gps_hdop))
      gps_hdop_wsum += (double)gps_hdop * w;

    gps_accum_samples++;
  }

  gpsMaybeFinalizeAverage();
}

static void gpsDumpToSerialTick()
{
  uint32_t now = millis();
  if (now - last_gps_print_ms < GPS_PRINT_PERIOD_MS)
    return;
  last_gps_print_ms = now;

  GpsQuality q = gpsQualityNow();
  Serial.printf("[GPS] q=%s sats=%lu hdop=%s alt=%s lat=%s lon=%s\n",
                gpsQualityLabel(q),
                (unsigned long)gps_sats,
                isnan(gps_hdop) ? "INV" : String(gps_hdop, 2).c_str(),
                isnan(gps_alt_m) ? "INV" : String(gps_alt_m, 1).c_str(),
                gps_has_fix ? String(gps_lat, 5).c_str() : "INV",
                gps_has_fix ? String(gps_lon, 5).c_str() : "INV");
}

// -------------------- OLED helpers --------------------
static void drawStatusOLED()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  GpsQuality q = gpsQualityNow();

  display.drawString(0, 0, "Heltec V3  GPS");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, gpsQualityLabel(q));
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  bool avg_fresh = gps_has_avg && ((millis() - gps_last_avg_ms) < (GPS_AVG_WINDOW_MS * 2));
  float showLat = avg_fresh ? gps_lat_avg : gps_lat;
  float showLon = avg_fresh ? gps_lon_avg : gps_lon;

  String latStr = (gps_has_fix || avg_fresh) ? String(showLat, 5) : String("-----.-----");
  String lonStr = (gps_has_fix || avg_fresh) ? String(showLon, 5) : String("-----.-----");

  display.drawString(0, 12, "Lat: " + latStr);
  display.drawString(0, 24, "Lon: " + lonStr);

  float showAlt = avg_fresh ? gps_alt_avg_m : gps_alt_m;
  float showHdop = avg_fresh ? gps_hdop_avg : gps_hdop;

  String altStr = (!isnan(showAlt)) ? (String(showAlt, 0) + "m") : String("--m");
  String hdopStr = (!isnan(showHdop)) ? String(showHdop, 2) : String("--");
  String satsStr = (gps_sats > 0) ? String(gps_sats) : String("--");

  display.drawString(0, 36, "Alt:" + altStr + " HDOP:" + hdopStr + " S:" + satsStr);

  // SD status line
  display.drawString(0, 48, String("SD: ") + (sd_ok ? "OK" : "FAIL") + "  TX#" + String(txCounter));

  // RX status / last packet stats
  if (!isnan(lastRSSI) && (millis() - last_rx_ms) < 10000)
    display.drawString(0, 56, "RX R:" + String(lastRSSI, 0) + " S:" + String(lastSNR, 1));
  else
    display.drawString(0, 56, String(millis() / 1000) + "s");

  display.display();
}

static void oledTick()
{
  uint32_t now = millis();
  if (now - last_oled_ms < OLED_PERIOD_MS)
    return;
  last_oled_ms = now;
  drawStatusOLED();
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
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));

  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

static void buildTelemetryPacket(TelemetryPacket &p)
{
  memset(&p, 0, sizeof(p));
  p.magic = TLMS_MAGIC;
  p.version = 1;

  const uint32_t now = millis();
  p.uptime_s = now / 1000;
  p.seq = (uint16_t)(txCounter & 0xFFFF);

  bool avg_fresh = gps_has_avg && ((now - gps_last_avg_ms) < (GPS_AVG_WINDOW_MS * 2));
  bool has_fix_fresh = gps_has_fix && ((now - gps_last_fix_ms) <= GPS_FIX_STALE_MS);

  float lat = avg_fresh ? gps_lat_avg : gps_lat;
  float lon = avg_fresh ? gps_lon_avg : gps_lon;
  float altm = avg_fresh ? gps_alt_avg_m : gps_alt_m;
  float hdop = avg_fresh ? gps_hdop_avg : gps_hdop;

  if (has_fix_fresh || avg_fresh)
    p.flags |= (1u << 0);
  if (avg_fresh)
    p.flags |= (1u << 1);
  // bit2 (charging) unused now

  p.gps_quality = (uint8_t)gpsQualityNow();

  if (!isnan(lat) && !isnan(lon) && (has_fix_fresh || avg_fresh))
  {
    p.lat_e7 = (int32_t)llround((double)lat * 1e7);
    p.lon_e7 = (int32_t)llround((double)lon * 1e7);
  }

  if (!isnan(altm))
  {
    double dm = (double)altm * 10.0;
    if (dm > 32767)
      dm = 32767;
    if (dm < -32768)
      dm = -32768;
    p.alt_dm = (int16_t)llround(dm);
  }
  else
  {
    p.alt_dm = (int16_t)0x7FFF;
  }

  if (!isnan(hdop))
  {
    double hc = (double)hdop * 100.0;
    if (hc < 0)
      hc = 0;
    if (hc > 65535)
      hc = 65535;
    p.hdop_c = (uint16_t)llround(hc);
  }
  else
  {
    p.hdop_c = 0xFFFF;
  }

  p.sats = (gps_sats > 255) ? 255 : (uint8_t)gps_sats;

  // INA removed -> keep these at 0 for now
  p.current_mA = 0;
  p.v_mV = 0;
  p.net_mAh_x1000 = 0;
  p.in_mAh_x1000 = 0;
  p.out_mAh_x1000 = 0;

  if (!isnan(lastRSSI) && (now - last_rx_ms) < 10000)
  {
    double r = lastRSSI;
    if (r > 32767)
      r = 32767;
    if (r < -32768)
      r = -32768;
    p.rssi_dBm_x1 = (int16_t)llround(r);

    double s10 = (double)lastSNR * 10.0;
    if (s10 > 32767)
      s10 = 32767;
    if (s10 < -32768)
      s10 = -32768;
    p.snr_dB_x10 = (int16_t)llround(s10);
  }
  else
  {
    p.rssi_dBm_x1 = (int16_t)0x7FFF;
    p.snr_dB_x10 = (int16_t)0x7FFF;
  }

  p.crc = 0;
  p.crc = crc16_ccitt((const uint8_t *)&p, sizeof(p) - sizeof(p.crc));
}

static void handleTX()
{
  uint32_t now = millis();

  bool tx_legal = (now - last_tx_ms) >= minimum_pause_ms;
  bool timeToTx = (PAUSE > 0) && (now - last_tx_ms) >= (uint32_t)PAUSE * 1000UL;
  bool buttonTx = button.isSingleClick();

  if (!(timeToTx || buttonTx))
    return;
  if (!tx_legal)
    return;

  radio.clearDio1Action();

  TelemetryPacket pkt;
  buildTelemetryPacket(pkt);

  heltec_led(50);
  tx_time_ms = millis();
  last_tx_start_ms = tx_time_ms;
  RADIOLIB(radio.transmit((uint8_t *)&pkt, sizeof(pkt)));
  tx_time_ms = millis() - tx_time_ms;
  heltec_led(0);

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    String s = String(millis()) + ",PING_FAIL," + String(_radiolib_status);
    Serial.printf("PING TX fail: %d\n", _radiolib_status);
    sdAppendLine(s);
  }
  else
  {
    Serial.printf("PING tx %u bytes seq=%u (air=%lums)\n",
                  (unsigned)sizeof(pkt), (unsigned)pkt.seq, (unsigned long)tx_time_ms);
    last_tx_seq = pkt.seq;

    String s = String(millis()) + ",PING_TX,seq=" + String(pkt.seq) + " bytes=" + String((unsigned)sizeof(pkt)) +
               " airMs=" + String((unsigned long)tx_time_ms);
    sdAppendLine(s);
  }

  minimum_pause_ms = tx_time_ms * 2;
  last_tx_ms = millis();
  txCounter++;

  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

static void logAckWithPingContext(const AckPacket &ack, float ack_rssi, float ack_snr)
{
  const uint32_t rtt_ms = (last_tx_start_ms > 0) ? (millis() - last_tx_start_ms) : 0;

  const char *ping_fe = (ack.ping_freqerr_Hz == 0x7FFFFFFF) ? "INV" : String((long)ack.ping_freqerr_Hz).c_str();

  String line;
  line.reserve(220);
  line += "PONG seq=";
  line += String((unsigned)ack.seq);
  line += " rtt=";
  line += String((unsigned long)rtt_ms);
  line += "ms | beaconTX{seq=";
  line += String((unsigned)last_tx_seq);
  line += "} | rxPing{R=";
  line += String((int)ack.ping_rssi_dBm_x1);
  line += "dBm S=";
  line += String((double)ack.ping_snr_dB_x10 / 10.0, 1);
  line += "dB FE=";
  line += ping_fe;
  line += "Hz len=";
  line += String((unsigned)ack.ping_len);
  line += " rxUp=";
  line += String((unsigned long)ack.rx_uptime_s);
  line += "s} | beaconRxAck{R=";
  line += String(ack_rssi, 1);
  line += "dBm S=";
  line += String(ack_snr, 1);
  line += "dB}";

  Serial.println(line);

  // CSV log entry
  sdAppendLine(String(millis()) + ",PONG," + line);
}

static void handleRX()
{
  if (!rxFlag)
    return;
  rxFlag = false;

  uint8_t buf[64] = {0};
  int16_t len = radio.getPacketLength(true);
  size_t toRead = (len > 0 && (size_t)len < sizeof(buf)) ? (size_t)len : sizeof(buf);
  radio.readData(buf, toRead);

  lastRSSI = radio.getRSSI();
  lastSNR = radio.getSNR();
  last_rx_ms = millis();

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    Serial.printf("RX read fail: %d RSSI=%.1f SNR=%.1f len=%d\n", _radiolib_status, lastRSSI, lastSNR, (int)len);
    sdAppendLine(String(millis()) + ",RX_FAIL,status=" + String(_radiolib_status) +
                 " rssi=" + String(lastRSSI, 1) + " snr=" + String(lastSNR, 1) + " len=" + String((int)len));
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    return;
  }

  if ((size_t)len == sizeof(AckPacket))
  {
    AckPacket ack;
    if (decodeAck(buf, sizeof(AckPacket), ack))
    {
      if (ack.seq == last_tx_seq)
        logAckWithPingContext(ack, lastRSSI, lastSNR);
      else
      {
        Serial.printf("PONG (out-of-order) seq=%u (expected %u) RSSI=%.1f SNR=%.1f\n",
                      (unsigned)ack.seq, (unsigned)last_tx_seq, lastRSSI, lastSNR);
        sdAppendLine(String(millis()) + ",PONG_OOO,seq=" + String((unsigned)ack.seq) +
                     " expected=" + String((unsigned)last_tx_seq) +
                     " rssi=" + String(lastRSSI, 1) + " snr=" + String(lastSNR, 1));
      }
    }
    else
    {
      Serial.printf("RX invalid ACK (crc/magic) RSSI=%.1f SNR=%.1f\n", lastRSSI, lastSNR);
      sdAppendLine(String(millis()) + ",ACK_BAD,rssi=" + String(lastRSSI, 1) + " snr=" + String(lastSNR, 1));
    }
  }
  else
  {
    Serial.printf("RX unknown len=%d RSSI=%.1f SNR=%.1f\n", (int)len, lastRSSI, lastSNR);
    sdAppendLine(String(millis()) + ",RX_UNKNOWN,len=" + String((int)len) +
                 " rssi=" + String(lastRSSI, 1) + " snr=" + String(lastSNR, 1));
  }

  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

// -------------------- Arduino entrypoints --------------------
void setup()
{
  Serial.begin(115200);

  heltec_setup();

  display.init();
  display.setFont(ArialMT_Plain_10);

  gpsInit();
  gpsResetAverager();
  radioInit();

  sdInit(); // <-- SD after core init; logs status to OLED

  drawStatusOLED();

  Serial.printf("TelemetryPacket size = %u bytes\n", (unsigned)sizeof(TelemetryPacket));
  Serial.printf("AckPacket size       = %u bytes\n", (unsigned)sizeof(AckPacket));

  sdAppendLine(String(millis()) + ",BOOT,beacon started");
}

void loop()
{
  heltec_loop();

  gpsPoll();
  gpsDumpToSerialTick();

  handleTX();
  handleRX();

  oledTick();
}
