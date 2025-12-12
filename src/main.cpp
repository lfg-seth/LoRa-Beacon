/**
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * - Raw LoRa TX/RX (no LoRaWAN)
 * - INA219 current sampling + EMA filter (SIGNED current supported)
 * - GPS (GT-U7) on UART pins 45/46
 *   - Serial GPS dump
 *   - OLED shows: HDOP, ALT, LAT/LON (AVERAGED)
 *   - GPS quality indicator: POOR / NORMAL / EXCELLENT
 * - Broadcast ALL stats as a binary struct over LoRa + CRC16
 */

#define HELTEC_POWER_BUTTON
#define HELTEC_WIRELESS_STICK
#include <heltec_unofficial.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGPSPlus.h>

// -------------------- LoRa settings --------------------
#define PAUSE 1           // seconds (0 = only button)
#define FREQUENCY 910.525 // MHz
#define BANDWIDTH 125.0   // kHz
#define SPREADING_FACTOR 9
#define TRANSMIT_POWER 10 // dBm

// -------------------- INA219 settings --------------------
static const uint32_t INA_SAMPLE_PERIOD_MS = 200;
static const float CURRENT_EMA_ALPHA = 0.15f;
static const float ZERO_DEADBAND_mA = 3.0f;

enum InaRangePreset
{
  INA_32V_2A,
  INA_32V_1A,
  INA_16V_400mA
};
static const InaRangePreset INA_PRESET = INA_32V_2A;

static const int INA_SDA_PIN = 6;
static const int INA_SCL_PIN = 7;

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

// -------------------- OLED timing --------------------
static const uint32_t OLED_PERIOD_MS = 250;

// -------------------- Globals --------------------
Adafruit_INA219 ina219;

volatile bool rxFlag = false;
String rxdata;

long txCounter = 0;
uint32_t last_tx_ms = 0;
uint32_t minimum_pause_ms = 0;
uint32_t tx_time_ms = 0;

// INA readings
static uint32_t last_ina_sample_ms = 0;
static float current_mA_raw = 0.0f; // signed
static float current_mA_filt = NAN; // signed
static float busV = 0.0f;
static float shunt_mV = 0.0f;
static float loadV = 0.0f;

// Accumulators (signed + split)
static double net_mAh = 0.0;
static double mAh_out = 0.0;
static double mAh_in = 0.0;
static double net_Wh = 0.0;

// RX stats
static float lastRSSI = NAN;
static float lastSNR = NAN;
static uint32_t last_rx_ms = 0;

// OLED tick
static uint32_t last_oled_ms = 0;

// -------------------- Telemetry packet --------------------
//
// Little-endian packed struct, 40 bytes total.
// Scaling:
//  - lat/lon: int32 = degrees * 1e7
//  - alt_dm:  int16 = meters * 10 (decimeters)
//  - hdop_c:  uint16 = HDOP * 100 (centi-HDOP)
//  - current_mA: int16 signed milliamps
//  - v_mV: uint16 millivolts
//  - net_mAh_x1000: int32 mAh * 1000
//  - in/out_mAh_x1000: uint32 mAh * 1000
//
enum GpsQuality : uint8_t
{
  GPS_NOFIX = 0,
  GPS_POOR = 1,
  GPS_NORMAL = 2,
  GPS_EXCELLENT = 3
};

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
  uint32_t uptime_s; // millis()/1000

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

// -------------------- ISR --------------------
void rx() { rxFlag = true; }

// -------------------- INA helpers --------------------
static void initINA219()
{
  Wire1.begin(INA_SDA_PIN, INA_SCL_PIN);

  if (!ina219.begin(&Wire1))
  {
    Serial.println("INA219 not found (Wire1). Check wiring/address.");
    return;
  }

  switch (INA_PRESET)
  {
  case INA_16V_400mA:
    ina219.setCalibration_16V_400mA();
    break;
  case INA_32V_1A:
    ina219.setCalibration_32V_1A();
    break;
  case INA_32V_2A:
  default:
    ina219.setCalibration_32V_2A();
    break;
  }

  last_ina_sample_ms = millis();
}

static void sampleAndIntegrateINA219()
{
  uint32_t now = millis();
  uint32_t dt_ms = now - last_ina_sample_ms;
  if (dt_ms < INA_SAMPLE_PERIOD_MS)
    return;
  last_ina_sample_ms = now;

  shunt_mV = ina219.getShuntVoltage_mV();
  busV = ina219.getBusVoltage_V();
  loadV = busV + (shunt_mV / 1000.0f);

  current_mA_raw = ina219.getCurrent_mA(); // signed

  if (fabsf(current_mA_raw) < ZERO_DEADBAND_mA)
    current_mA_raw = 0.0f;

  if (isnan(current_mA_filt))
    current_mA_filt = current_mA_raw;
  current_mA_filt = (CURRENT_EMA_ALPHA * current_mA_raw) + ((1.0f - CURRENT_EMA_ALPHA) * current_mA_filt);

  const double dt_hours = (double)dt_ms / 3600000.0;

  net_mAh += (double)current_mA_filt * dt_hours;
  if (current_mA_filt >= 0.0f)
    mAh_out += (double)current_mA_filt * dt_hours;
  else
    mAh_in += (double)(-current_mA_filt) * dt_hours;

  const double watts = ((double)loadV * (double)current_mA_filt) / 1000.0; // signed
  net_Wh += watts * dt_hours;
}

static const char *powerStateLabel()
{
  if (fabsf(current_mA_filt) < 1.0f)
    return "IDLE";
  if (current_mA_filt < 0.0f)
    return "CHG";
  return "DIS";
}

// -------------------- GPS helpers --------------------
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

  if (gps_has_avg)
  {
    Serial.printf("[GPS-AVG] samples=%lu age=%lums lat=%.5f lon=%.5f alt=%.1f hdop=%.2f\n",
                  (unsigned long)gps_avg_samples,
                  (unsigned long)(now - gps_last_avg_ms),
                  gps_lat_avg, gps_lon_avg, gps_alt_avg_m, gps_hdop_avg);
  }
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

  String iStr = "I:" + String(current_mA_filt, 0) + "mA V:" + String(loadV, 2) + "V";
  display.drawString(0, 48, iStr);

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 48, powerStateLabel());
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  if (!isnan(lastRSSI) && (millis() - last_rx_ms) < 10000)
  {
    display.drawString(0, 56, "RX R:" + String(lastRSSI, 0) + " S:" + String(lastSNR, 1));
  }
  else
  {
    display.drawString(0, 56, "TX#" + String(txCounter) + " " + String(millis() / 1000) + "s");
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

  // Choose averaged GPS if fresh, else raw
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
  if (current_mA_filt < -1.0f)
    p.flags |= (1u << 2); // charging

  GpsQuality q = gpsQualityNow();
  p.gps_quality = (uint8_t)q;

  // Scaled fields (guard NaNs)
  if (!isnan(lat) && !isnan(lon) && (has_fix_fresh || avg_fresh))
  {
    p.lat_e7 = (int32_t)llround((double)lat * 1e7);
    p.lon_e7 = (int32_t)llround((double)lon * 1e7);
  }
  else
  {
    p.lat_e7 = 0;
    p.lon_e7 = 0;
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

  // Po
