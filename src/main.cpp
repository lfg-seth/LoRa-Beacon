/**
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * - Raw LoRa TX/RX (no LoRaWAN)
 * - INA219 current sampling + EMA filter (SIGNED current supported)
 * - GPS (GT-U7) on UART pins 45/46
 *   - Serial GPS dump
 *   - OLED shows: HDOP, ALT, LAT/LON (AVERAGED)
 *   - GPS quality indicator: POOR / NORMAL / EXCELLENT
 *
 * Averaging:
 * - Weighted average (weights based on HDOP) over a time window
 * - Only accumulates samples when GPS quality is at least NORMAL
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

// Fix staleness: if no valid location update in this time, treat as no-fix for OLED/quality
static const uint32_t GPS_FIX_STALE_MS = 5000;

// Quality thresholds (tune if you want)
static const uint32_t GPS_SATS_NORMAL = 7;
static const uint32_t GPS_SATS_EXCELLENT = 10;
static const float GPS_HDOP_NORMAL = 3.0f;
static const float GPS_HDOP_EXCELLENT = 1.5f;

// Averaging window
static const uint32_t GPS_AVG_WINDOW_MS = 120000; // 2 minutes
static const uint32_t GPS_AVG_MIN_SAMPLES = 20;   // require at least N good samples before updating avg

// Cached latest raw values (for gating/serial)
static bool gps_has_fix = false;
static float gps_lat = NAN, gps_lon = NAN, gps_alt_m = NAN, gps_hdop = NAN;
static uint32_t gps_sats = 0;
static uint32_t gps_last_fix_ms = 0;

// Averaged values (what OLED will show)
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
enum GpsQuality
{
  GPS_NOFIX,
  GPS_POOR,
  GPS_NORMAL,
  GPS_EXCELLENT
};

static GpsQuality gpsQualityNow()
{
  if (!gps_has_fix)
    return GPS_NOFIX;
  if ((millis() - gps_last_fix_ms) > GPS_FIX_STALE_MS)
    return GPS_NOFIX;

  // If we don't have hdop/sats, be conservative
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
  bool window_done = (now - gps_window_start_ms) >= GPS_AVG_WINDOW_MS;

  if (!window_done)
    return;
  // Window ended: compute if enough samples, then reset window either way
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

  // Update raw cache when valid
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

  // Accumulate into averager if quality is NORMAL or better
  GpsQuality q = gpsQualityNow();
  if ((q == GPS_NORMAL || q == GPS_EXCELLENT) && !isnan(gps_hdop) && gps_hdop > 0.0f)
  {
    // HDOP-weighted average: stronger weight for better (lower) HDOP
    // Clamp HDOP to avoid insane weights if hdop reports tiny values
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

  Serial.printf("[GPS] q=%s fix=%s sats=%lu hdop=%s alt=%s lat=%s lon=%s age=%lums\n",
                gpsQualityLabel(q),
                gps.location.isValid() ? "YES" : "NO",
                (unsigned long)gps_sats,
                isnan(gps_hdop) ? "INV" : String(gps_hdop, 2).c_str(),
                isnan(gps_alt_m) ? "INV" : String(gps_alt_m, 1).c_str(),
                gps_has_fix ? String(gps_lat, 5).c_str() : "INV",
                gps_has_fix ? String(gps_lon, 5).c_str() : "INV",
                (unsigned long)(gps.location.isValid() ? gps.location.age() : 0));

  if (gps_has_avg)
  {
    Serial.printf("[GPS-AVG] samples=%lu age=%lums lat=%.5f lon=%.5f alt=%.1f hdop=%.2f\n",
                  (unsigned long)gps_avg_samples,
                  (unsigned long)(now - gps_last_avg_ms),
                  gps_lat_avg, gps_lon_avg, gps_alt_avg_m, gps_hdop_avg);
  }
}

// -------------------- OLED helpers (ThingPulse style) --------------------
static void drawStatusOLED()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // OLED layout (64px):
  // y=0   title + GPS quality on right
  // y=12  lat (avg if available)
  // y=24  lon (avg if available)
  // y=36  alt + hdop + sats (avg hdop/alt if available)
  // y=48  current + power state
  // y=56  RX recent else TX# + uptime

  GpsQuality q = gpsQualityNow();

  display.drawString(0, 0, "Heltec V3  GPS");

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, gpsQualityLabel(q));
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Prefer averaged position if we have it and it's not too old
  bool avg_fresh = gps_has_avg && ((millis() - gps_last_avg_ms) < (GPS_AVG_WINDOW_MS * 2));
  float showLat = avg_fresh ? gps_lat_avg : gps_lat;
  float showLon = avg_fresh ? gps_lon_avg : gps_lon;

  String latStr = (gps_has_fix || avg_fresh) ? String(showLat, 5) : String("-----.-----");
  String lonStr = (gps_has_fix || avg_fresh) ? String(showLon, 5) : String("-----.-----");

  display.drawString(0, 12, "Lat: " + latStr);
  display.drawString(0, 24, "Lon: " + lonStr);

  // Alt/HDOP: show averaged if fresh, else raw
  float showAlt = avg_fresh ? gps_alt_avg_m : gps_alt_m;
  float showHdop = avg_fresh ? gps_hdop_avg : gps_hdop;

  String altStr = (!isnan(showAlt)) ? (String(showAlt, 0) + "m") : String("--m");
  String hdopStr = (!isnan(showHdop)) ? String(showHdop, 2) : String("--");
  String satsStr = (gps_sats > 0) ? String(gps_sats) : String("--");

  String line3 = "Alt:" + altStr + " HDOP:" + hdopStr + " S:" + satsStr;
  display.drawString(0, 36, line3);

  // Current + state
  String iStr = "I:" + String(current_mA_filt, 0) + "mA V:" + String(loadV, 2) + "V";
  display.drawString(0, 48, iStr);

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 48, powerStateLabel());
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Bottom line
  if (!isnan(lastRSSI) && (millis() - last_rx_ms) < 10000)
  {
    display.drawString(0, 56, "RX R:" + String(lastRSSI, 0) + " S:" + String(lastSNR, 1));
  }
  else
  {
    String b = "TX#" + String(txCounter) + " " + String(millis() / 1000) + "s";
    display.drawString(0, 56, b);
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

  heltec_led(50);
  tx_time_ms = millis();
  RADIOLIB(radio.transmit(String(txCounter).c_str()));
  tx_time_ms = millis() - tx_time_ms;
  heltec_led(0);

  if (_radiolib_status != RADIOLIB_ERR_NONE)
    Serial.printf("TX fail: %d\n", _radiolib_status);

  minimum_pause_ms = tx_time_ms * 2;
  last_tx_ms = millis();
  txCounter++;

  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

static void handleRX()
{
  if (!rxFlag)
    return;
  rxFlag = false;

  radio.readData(rxdata);
  if (_radiolib_status == RADIOLIB_ERR_NONE)
  {
    lastRSSI = radio.getRSSI();
    lastSNR = radio.getSNR();
    last_rx_ms = millis();

    Serial.printf("RX [%s] RSSI=%.1f SNR=%.1f\n", rxdata.c_str(), lastRSSI, lastSNR);
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

  initINA219();
  gpsInit();
  radioInit();

  gpsResetAverager();
  drawStatusOLED();
}

void loop()
{
  heltec_loop();

  sampleAndIntegrateINA219();

  gpsPoll();
  gpsDumpToSerialTick();

  handleTX();
  handleRX();

  oledTick();
}
