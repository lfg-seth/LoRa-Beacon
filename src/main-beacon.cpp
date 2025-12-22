/**
 * Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262) - BEACON (mobile logger)
 * - Raw LoRa TX/RX (no LoRaWAN)
 * - GPS (GT-U7) on UART pins 45/46
 * - microSD logging over custom SPI pins (4/5/2 + CS=6)
 * - Broadcast TelemetryPacket over LoRa + CRC16 (PING)
 * - Multiple receivers respond with AckPacket (PONG). Beacon listens for a window
 *   and logs *each* valid PONG to SD.
 * - Rich CSV logging: GPS, VBAT, radio settings, TX airtime, per-receiver RTT,
 *   RSSI/SNR/FreqError, etc.
 */

#define HELTEC_POWER_BUTTON
#define HELTEC_WIRELESS_STICK
#include <heltec_unofficial.h>

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// -------------------- LoRa settings --------------------
#define PAUSE 1           // seconds (0 = only button)
#define FREQUENCY 910.525 // MHz
#define BANDWIDTH 125.0   // kHz
#define SPREADING_FACTOR 9
#define TRANSMIT_POWER 22 // dBm

// -------------------- GPS (GT-U7) --------------------
static const int GPS_RX_PIN = 39;    // ESP32 RX  <- GPS TX
static const int GPS_TX_PIN = 38;    // ESP32 TX  -> GPS RX (optional)
static const int GPS_POWER_PIN = 34; // GPIO to control GPS power (if connected)
static const int GPS_WAKE = 40;      // GPIO to control GPS wake (if connected)
static const int GPS_PPS_PIN = 41;   // GPIO to read GPS PPS (if connected)
static const int GPS_RESET_PIN = 42; // GPIO to reset GPS (if connected)
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

static bool gps_time_valid = false;
static uint32_t gps_time_last_ms = 0;

static int gps_year = 0, gps_mon = 0, gps_day = 0;
static int gps_hour = 0, gps_min = 0, gps_sec = 0, gps_centi = 0;

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

// -------------------- Periodic STATUS logging --------------------
static const uint32_t STATUS_LOG_PERIOD_MS = 1000;
static uint32_t last_status_log_ms = 0;

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

// --------- Last PONG metrics (decoded + measured) ----------
static bool last_pong_valid = false;
static uint32_t last_pong_ms = 0;
static uint16_t last_pong_seq = 0;

static float last_ping_rssi_rx = NAN;           // receiver-reported RSSI of our PING
static float last_ping_snr_rx = NAN;            // receiver-reported SNR of our PING
static int32_t last_ping_fe_hz_rx = 0x7FFFFFFF; // receiver-reported freq error on our PING
static uint16_t last_ping_len_rx = 0;

static float last_pong_rssi_beacon = NAN; // beacon-measured RSSI of PONG
static float last_pong_snr_beacon = NAN;  // beacon-measured SNR of PONG
static uint32_t last_pong_rtt_ms = 0;

// OLED tick
static uint32_t last_oled_ms = 0;

// Track last TX packet so we can correlate ACKs
static uint16_t last_tx_seq = 0;
static uint32_t last_tx_start_ms = 0;

// -------------------- Battery (VBAT) --------------------
// Primary: heltec_vbat() if provided by heltec_unofficial.
// Fallback: ADC pin method (adjust to your board if needed).
static const int VBAT_ADC_PIN = 1;      // Common on Heltec V3 variants, may differ
static const float VBAT_DIVIDER = 2.0f; // If VBAT is halved before ADC (common). Adjust if wrong.
static const float ADC_VREF = 3.3f;     // ESP32-S3 ADC reference-ish (approx; calibration varies)
static const float ADC_MAX = 4095.0f;

static float readVBAT()
{
  // If heltec_unofficial provides heltec_vbat(), use it.
  // Many Heltec examples expose this helper.
  return heltec_vbat();

  // // Fallback ADC method
  // // NOTE: You may need analogReadMilliVolts() depending on your core version.
  // uint16_t raw = analogRead(VBAT_ADC_PIN);
  // float v_adc = (raw / ADC_MAX) * ADC_VREF;
  // float v_bat = v_adc * VBAT_DIVIDER;
  // return v_bat;
}

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

  uint32_t receiver_id;       // receiver unique id (e.g. MCU DEVICEID hash)
  uint16_t receiver_delay_ms; // how long receiver waited before TX (anti-collision)

  uint32_t rx_uptime_s; // receiver uptime when ACK built

  int16_t ping_rssi_dBm_x1; // RSSI seen by receiver on PING
  int16_t ping_snr_dB_x10;  // SNR*10 seen by receiver on PING
  int32_t ping_freqerr_Hz;  // frequency error seen by receiver on PING (if available), else 0x7FFFFFFF
  uint16_t ping_len;        // length of received PING

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
    sd_ok = false;
    return;
  }
  f.println(line);
  f.close();
}

static void sdInit()
{
  SDSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SDSPI))
  {
    Serial.println("SD init failed.");
    sd_ok = false;
    return;
  }

  sd_ok = true;
  Serial.println("SD init OK.");

  if (!SD.exists(SD_PATH))
  {
    File f = SD.open(SD_PATH, FILE_WRITE);
    if (f)
    {
      // Rich CSV header (stable columns)
      f.println(
          "gps_utc_valid,gps_utc_year,gps_utc_mon,gps_utc_day,gps_utc_hour,gps_utc_min,gps_utc_sec,gps_utc_centis,gps_utc_age_ms,"
          "ms,event,"
          "uptime_s,seq,"
          "vbat_V,"
          "gps_has_fix,gps_avg_used,gps_quality,gps_sats,gps_hdop,gps_alt_m,gps_lat,gps_lon,"
          "gps_avg_samples,gps_avg_hdop,gps_avg_alt_m,gps_avg_lat,gps_avg_lon,"
          "radio_freq_MHz,radio_bw_kHz,radio_sf,radio_txpwr_dBm,"
          "tx_air_ms,min_pause_ms,"
          "rx_rssi_dBm,rx_snr_dB,rx_age_ms,rx_len,"
          "pong_receiver_id,pong_receiver_delay_ms,pong_rx_uptime_s,pong_ping_rssi_dBm,pong_ping_snr_dB,pong_ping_freqerr_Hz,pong_ping_len,pong_rtt_ms,"
          "status");
      f.close();
    }
  }
}

// -------------------- GPS helpers --------------------

void gpsPowerInit()
{
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, LOW); // Power ON
  delay(50);
  Serial.printf("GPS power ON (PIN %d)\n", GPS_POWER_PIN);
}
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
  gpsPowerInit();
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

  if (gps.date.isValid() && gps.time.isValid())
  {
    gps_year = gps.date.year();
    gps_mon = gps.date.month();
    gps_day = gps.date.day();

    gps_hour = gps.time.hour();
    gps_min = gps.time.minute();
    gps_sec = gps.time.second();
    gps_centi = gps.time.centisecond();

    gps_time_valid = true;
    gps_time_last_ms = millis();
  }
  else if (gps_time_valid && (millis() - gps_time_last_ms) > GPS_FIX_STALE_MS)
  {
    gps_time_valid = false;
  }

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

  // Tune these thresholds to your field results.
  // LoRa is usually "useable" even with negative SNR at higher SF,
  // but for a simple label this works well.
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

static void drawStatusOLED()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  GpsQuality q = gpsQualityNow();

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, gpsQualityLabel(q));
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  uint32_t now = millis();
  bool avg_fresh = gps_has_avg && ((now - gps_last_avg_ms) < (GPS_AVG_WINDOW_MS * 2));
  float showLat = avg_fresh ? gps_lat_avg : gps_lat;
  float showLon = avg_fresh ? gps_lon_avg : gps_lon;

  String latStr = (gps_has_fix || avg_fresh) ? String(showLat, 5) : String("-----.-----");
  String lonStr = (gps_has_fix || avg_fresh) ? String(showLon, 5) : String("-----.-----");

  float vbat = readVBAT();

  display.drawString(0, 0, "Lat: " + latStr + (sd_ok ? " SD" : " NOSD"));
  display.drawString(0, 12, "Lon: " + lonStr + " V:" + String(vbat, 2));

  float showAlt = avg_fresh ? gps_alt_avg_m : gps_alt_m;
  float showHdop = avg_fresh ? gps_hdop_avg : gps_hdop;

  String altStr = (!isnan(showAlt)) ? (String(showAlt, 0) + "m") : String("--m");
  String hdopStr = (!isnan(showHdop)) ? String(showHdop, 2) : String("--");
  String satsStr = (gps_sats > 0) ? String(gps_sats) : String("--");

  display.drawString(0, 24, "Alt:" + altStr + " HDOP:" + hdopStr + " S:" + satsStr);

  // --- PING stats (receiver reported it received) ---
  // Only valid if we've received at least one good PONG recently
  bool pong_fresh = last_pong_valid && ((now - last_pong_ms) < 15000);

  if (pong_fresh)
  {
    // PING quality as reported by receiver (what RX saw from our PING)
    SigQuality pingQ = signalQuality(last_ping_rssi_rx, last_ping_snr_rx);

    // PONG quality as measured locally (what we saw from their PONG)
    SigQuality pongQ = signalQuality(last_pong_rssi_beacon, last_pong_snr_beacon);

    display.drawString(
        0, 36,
        "R:" + String(last_ping_rssi_rx, 0) +
            " S:" + String(last_ping_snr_rx, 1) +
            String(sigQualityLabel(pingQ)));

    display.drawString(
        0, 48,
        "R:" + String(last_pong_rssi_beacon, 0) +
            " S:" + String(last_pong_snr_beacon, 1) +
            String(sigQualityLabel(pongQ)));
  }
  else
  {
    // Fallback: show something useful while waiting for first PONG
    display.drawString(0, 36, "PING@RX");
    display.drawString(0, 48, "PONG@ME");
  }

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

  // INA removed -> keep these at 0
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

// -------------------- CSV logging helpers --------------------
static void appendCsvRow(
    const char *event,
    uint32_t uptime_s,
    uint16_t seq,
    const char *status,
    // RX basics (beacon side)
    int rx_len,
    float rx_rssi,
    float rx_snr,
    uint32_t rx_age_ms,
    // TX basics
    uint32_t tx_air_ms_local,
    uint32_t min_pause_ms_local,
    // PONG fields (or sentinel)
    const AckPacket *ack,
    uint32_t pong_rtt_ms)
{
  float vbat = readVBAT();
  uint32_t now = millis();

  bool avg_fresh = gps_has_avg && ((now - gps_last_avg_ms) < (GPS_AVG_WINDOW_MS * 2));
  bool has_fix_fresh = gps_has_fix && ((now - gps_last_fix_ms) <= GPS_FIX_STALE_MS);

  float showLat = avg_fresh ? gps_lat_avg : gps_lat;
  float showLon = avg_fresh ? gps_lon_avg : gps_lon;
  float showAlt = avg_fresh ? gps_alt_avg_m : gps_alt_m;
  float showHdop = avg_fresh ? gps_hdop_avg : gps_hdop;

  GpsQuality q = gpsQualityNow();

  // Build CSV line with consistent columns
  String line;
  line.reserve(640);

  uint32_t utc_age = gps_time_valid ? (millis() - gps_time_last_ms) : 0;

  line += String(gps_time_valid ? 1 : 0);
  line += ",";
  line += String(gps_year);
  line += ",";
  line += String(gps_mon);
  line += ",";
  line += String(gps_day);
  line += ",";
  line += String(gps_hour);
  line += ",";
  line += String(gps_min);
  line += ",";
  line += String(gps_sec);
  line += ",";
  line += String(gps_centi);
  line += ",";
  line += String((unsigned long)(gps_time_valid ? utc_age : 0));
  line += ",";

  line += String(now);
  line += ",";
  line += event;
  line += ",";
  line += String(uptime_s);
  line += ",";
  line += String((unsigned)seq);
  line += ",";

  line += String(vbat, 3);
  line += ",";

  line += String(has_fix_fresh ? 1 : 0);
  line += ",";
  line += String(avg_fresh ? 1 : 0);
  line += ",";
  line += String((unsigned)q);
  line += ",";
  line += String((unsigned long)gps_sats);
  line += ",";
  line += (isnan(showHdop) ? "" : String(showHdop, 2));
  line += ",";
  line += (isnan(showAlt) ? "" : String(showAlt, 1));
  line += ",";
  line += ((has_fix_fresh || avg_fresh) && !isnan(showLat) ? String(showLat, 7) : "");
  line += ",";
  line += ((has_fix_fresh || avg_fresh) && !isnan(showLon) ? String(showLon, 7) : "");
  line += ",";

  line += String((unsigned long)gps_avg_samples);
  line += ",";
  line += (isnan(gps_hdop_avg) ? "" : String(gps_hdop_avg, 2));
  line += ",";
  line += (isnan(gps_alt_avg_m) ? "" : String(gps_alt_avg_m, 1));
  line += ",";
  line += (gps_has_avg ? String(gps_lat_avg, 7) : "");
  line += ",";
  line += (gps_has_avg ? String(gps_lon_avg, 7) : "");
  line += ",";

  line += String(FREQUENCY, 3);
  line += ",";
  line += String(BANDWIDTH, 1);
  line += ",";
  line += String(SPREADING_FACTOR);
  line += ",";
  line += String(TRANSMIT_POWER);
  line += ",";

  line += String((unsigned long)tx_air_ms_local);
  line += ",";
  line += String((unsigned long)min_pause_ms_local);
  line += ",";

  if (!isnan(rx_rssi))
    line += String(rx_rssi, 1);
  line += ",";
  if (!isnan(rx_snr))
    line += String(rx_snr, 1);
  line += ",";
  line += String((unsigned long)rx_age_ms);
  line += ",";
  line += String(rx_len);
  line += ",";

  // PONG fields
  if (ack)
  {
    line += String((unsigned long)ack->receiver_id);
    line += ",";
    line += String((unsigned long)ack->receiver_delay_ms);
    line += ",";
    line += String((unsigned long)ack->rx_uptime_s);
    line += ",";
    line += String((int)ack->ping_rssi_dBm_x1);
    line += ",";
    line += String((double)ack->ping_snr_dB_x10 / 10.0, 1);
    line += ",";
    if (ack->ping_freqerr_Hz == 0x7FFFFFFF)
      line += "";
    else
      line += String((long)ack->ping_freqerr_Hz);
    line += ",";
    line += String((unsigned)ack->ping_len);
    line += ",";
    line += String((unsigned long)pong_rtt_ms);
    line += ",";
  }
  else
  {
    // Empty columns when no ack
    line += ",,,,,,,,";
  }

  line += status ? status : "";
  sdAppendLine(line);
}

static void logStatusTick()
{
  uint32_t now = millis();
  if (now - last_status_log_ms < STATUS_LOG_PERIOD_MS)
    return;
  last_status_log_ms = now;

  uint32_t uptime_s = now / 1000;

  uint32_t rx_age = (!isnan(lastRSSI) ? (now - last_rx_ms) : 0xFFFFFFFF);

  // Serial breadcrumb
  Serial.printf("[STATUS] t=%lus vbat=%.2f gps=%s sats=%lu hdop=%s lat=%s lon=%s lastRxAge=%lums rssi=%s snr=%s\n",
                (unsigned long)uptime_s,
                readVBAT(),
                gpsQualityLabel(gpsQualityNow()),
                (unsigned long)gps_sats,
                isnan(gps_hdop) ? "INV" : String(gps_hdop, 2).c_str(),
                gps_has_fix ? String(gps_lat, 5).c_str() : "INV",
                gps_has_fix ? String(gps_lon, 5).c_str() : "INV",
                (unsigned long)(rx_age == 0xFFFFFFFF ? 0 : rx_age),
                isnan(lastRSSI) ? "INV" : String(lastRSSI, 1).c_str(),
                isnan(lastSNR) ? "INV" : String(lastSNR, 1).c_str());

  appendCsvRow(
      "STATUS",
      uptime_s,
      last_tx_seq,
      sd_ok ? "ok" : "sd_fail",
      -1,
      lastRSSI,
      lastSNR,
      (rx_age == 0xFFFFFFFF ? 0 : rx_age),
      tx_time_ms,
      minimum_pause_ms,
      nullptr,
      0);
}

// -------------------- TX / RX --------------------
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

  uint32_t uptime_s = millis() / 1000;
  uint16_t seq = pkt.seq;

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    Serial.printf("PING TX fail: %d\n", _radiolib_status);

    String status = "radiolib=";
    status += _radiolib_status;

    appendCsvRow(
        "PING_FAIL",
        uptime_s,
        seq,
        status.c_str(),
        -1,
        lastRSSI,
        lastSNR,
        (!isnan(lastRSSI) ? (millis() - last_rx_ms) : 0),
        tx_time_ms,
        minimum_pause_ms,
        nullptr,
        0);
  }
  else
  {
    Serial.printf("PING tx %u bytes seq=%u (air=%lums)\n",
                  (unsigned)sizeof(pkt), (unsigned)pkt.seq, (unsigned long)tx_time_ms);

    last_tx_seq = pkt.seq;

    appendCsvRow(
        "PING_TX",
        uptime_s,
        seq,
        "ok",
        (int)sizeof(pkt),
        lastRSSI,
        lastSNR,
        (!isnan(lastRSSI) ? (millis() - last_rx_ms) : 0),
        tx_time_ms,
        minimum_pause_ms,
        nullptr,
        0);
  }

  minimum_pause_ms = tx_time_ms * 2;
  last_tx_ms = millis();
  txCounter++;

  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

static void logAckWithPingContext(const AckPacket &ack, float ack_rssi, float ack_snr)
{

  // Cache “PING stats” (receiver reported) + “PONG stats” (beacon measured)
  const uint32_t now = millis();
  const uint32_t rtt_ms = (last_tx_start_ms > 0) ? (now - last_tx_start_ms) : 0;

  last_pong_valid = true;
  last_pong_ms = millis();
  last_pong_seq = ack.seq;

  last_ping_rssi_rx = (float)ack.ping_rssi_dBm_x1;
  last_ping_snr_rx = (float)ack.ping_snr_dB_x10 / 10.0f;
  last_ping_fe_hz_rx = ack.ping_freqerr_Hz;
  last_ping_len_rx = ack.ping_len;

  last_pong_rssi_beacon = ack_rssi;
  last_pong_snr_beacon = ack_snr;
  last_pong_rtt_ms = rtt_ms;

  String pingFeStr = (ack.ping_freqerr_Hz == 0x7FFFFFFF) ? String("INV") : String((long)ack.ping_freqerr_Hz);

  String pretty;
  pretty.reserve(240);
  pretty += "PONG seq=";
  pretty += String((unsigned)ack.seq);
  pretty += " from=";
  pretty += String((unsigned long)ack.receiver_id);
  pretty += " dly=";
  pretty += String((unsigned long)ack.receiver_delay_ms);
  pretty += "ms";
  pretty += " rtt=";
  pretty += String((unsigned long)rtt_ms);
  pretty += "ms | ";
  pretty += "rxPing{R=";
  pretty += String((int)ack.ping_rssi_dBm_x1);
  pretty += "dBm ";
  pretty += "S=";
  pretty += String((double)ack.ping_snr_dB_x10 / 10.0, 1);
  pretty += "dB ";
  pretty += "FE=";
  pretty += pingFeStr;
  pretty += "Hz ";
  pretty += "len=";
  pretty += String((unsigned)ack.ping_len);
  pretty += " rxUp=";
  pretty += String((unsigned long)ack.rx_uptime_s);
  pretty += "s} | ";
  pretty += "beaconRxAck{R=";
  pretty += String(ack_rssi, 1);
  pretty += "dBm S=";
  pretty += String(ack_snr, 1);
  pretty += "dB}";

  Serial.println(pretty);

  // Rich CSV row for the PONG
  uint32_t uptime_s = now / 1000;

  appendCsvRow(
      "PONG",
      uptime_s,
      ack.seq,
      "ok",
      (int)sizeof(AckPacket),
      ack_rssi,
      ack_snr,
      0,
      tx_time_ms,
      minimum_pause_ms,
      &ack,
      rtt_ms);
}

static void handleRX()
{
  if (!rxFlag)
    return;
  rxFlag = false;

  uint8_t buf[128] = {0};
  int16_t len = radio.getPacketLength(true);

  size_t toRead = 0;
  if (len > 0)
    toRead = (len < (int)sizeof(buf)) ? (size_t)len : sizeof(buf);
  else
    toRead = sizeof(buf);

  radio.readData(buf, toRead);

  lastRSSI = radio.getRSSI();
  lastSNR = radio.getSNR();
  last_rx_ms = millis();

  if (_radiolib_status != RADIOLIB_ERR_NONE)
  {
    Serial.printf("RX read fail: %d RSSI=%.1f SNR=%.1f len=%d\n", _radiolib_status, lastRSSI, lastSNR, (int)len);

    char status[32];
    snprintf(status, sizeof(status), "radiolib=%d", _radiolib_status);

    appendCsvRow(
        "RX_FAIL",
        millis() / 1000,
        last_tx_seq,
        status,
        (int)len,
        lastRSSI,
        lastSNR,
        0,
        tx_time_ms,
        minimum_pause_ms,
        nullptr,
        0);

    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    return;
  }

  if ((size_t)len == sizeof(AckPacket))
  {
    AckPacket ack;
    if (decodeAck(buf, sizeof(AckPacket), ack))
    {
      if (ack.seq == last_tx_seq)
      {
        logAckWithPingContext(ack, lastRSSI, lastSNR);
      }
      else
      {
        Serial.printf("PONG (out-of-order) seq=%u (expected %u) RSSI=%.1f SNR=%.1f\n",
                      (unsigned)ack.seq, (unsigned)last_tx_seq, lastRSSI, lastSNR);

        char status[32];
        snprintf(status, sizeof(status), "expected=%u", (unsigned)last_tx_seq);

        appendCsvRow(
            "PONG_OOO",
            millis() / 1000,
            ack.seq,
            status,
            (int)sizeof(AckPacket),
            lastRSSI,
            lastSNR,
            0,
            tx_time_ms,
            minimum_pause_ms,
            &ack,
            0);
      }
    }
    else
    {
      Serial.printf("RX invalid ACK (crc/magic) RSSI=%.1f SNR=%.1f\n", lastRSSI, lastSNR);

      appendCsvRow(
          "ACK_BAD",
          millis() / 1000,
          last_tx_seq,
          "crc_or_magic",
          (int)len,
          lastRSSI,
          lastSNR,
          0,
          tx_time_ms,
          minimum_pause_ms,
          nullptr,
          0);
    }
  }
  else
  {
    Serial.printf("RX unknown len=%d RSSI=%.1f SNR=%.1f\n", (int)len, lastRSSI, lastSNR);

    appendCsvRow(
        "RX_UNKNOWN",
        millis() / 1000,
        last_tx_seq,
        "unexpected_len",
        (int)len,
        lastRSSI,
        lastSNR,
        0,
        tx_time_ms,
        minimum_pause_ms,
        nullptr,
        0);
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

  // If ADC fallback is used, make sure pin is configured
  // analogReadResolution(12);
  // pinMode(VBAT_ADC_PIN, INPUT);

  gpsInit();
  gpsResetAverager();
  radioInit();
  sdInit();

  drawStatusOLED();

  Serial.printf("TelemetryPacket size = %u bytes\n", (unsigned)sizeof(TelemetryPacket));
  Serial.printf("AckPacket size       = %u bytes\n", (unsigned)sizeof(AckPacket));

  sdAppendLine(String(millis()) + ",BOOT,0,0," + String(readVBAT(), 3) + ",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,boot");
}

void loop()
{
  heltec_loop();

  gpsPoll();
  gpsDumpToSerialTick();

  handleTX();
  handleRX();

  logStatusTick();
  oledTick();
}
