/**
 * RAK4631 (nRF52840 + SX1262) - RECEIVER (can have many)
 *
 * Raw LoRa (no LoRaWAN):
 *   - Receives TelemetryPacket (PING)
 *   - Replies with AckPacket (PONG) containing:
 *       * receiver_id (unique per receiver)
 *       * receiver_delay_ms (anti-collision backoff used)
 *       * receiver uptime
 *       * receiver-measured RSSI/SNR/FreqErr/Len of the PING
 *
 * Notes:
 *   - This is meant to work with your Heltec V4 beacon sketch (same packet structs + CRC).
 *   - The RAK4631 uses SX1262; the classic "LoRa by Sandeep Mistry" library won't work for SX126x.
 *     Use RadioLib.
 */

#include <Arduino.h>
#include <RadioLib.h>

// -------------------- Radio settings (MUST match beacon) --------------------
#define FREQUENCY 910.525 // MHz
#define BANDWIDTH 125.0   // kHz
#define SPREADING_FACTOR 9
// -------------------- Receiver ID (unique per receiver) --------------------
#define RECEIVERID 0x0001u                  // change to a unique value per receiver
#define DELAY_MS (RECEIVERID * 350u + 200u) // Anti-collision delay based on receiver ID
// Anti-collision: receivers wait a deterministic delay before replying
static const uint16_t MAX_PONG_DELAY_MS = 300; // spread replies across this window

// -------------------- CRC16 (must match beacon) --------------------
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

// -------------------- ACK packet (PONG) (MUST match beacon) --------------------
struct __attribute__((packed)) AckPacket
{
  uint32_t magic;  // 'ACK1' = 0x314B4341 (little-endian)
  uint8_t version; // 1
  uint8_t flags;   // reserved
  uint16_t seq;    // echoed TelemetryPacket.seq

  uint32_t receiver_id;       // unique per receiver
  uint16_t receiver_delay_ms; // anti-collision delay used

  uint32_t rx_uptime_s; // receiver uptime when ACK built

  int16_t ping_rssi_dBm_x1; // RSSI seen on PING
  int16_t ping_snr_dB_x10;  // SNR*10 seen on PING
  int32_t ping_freqerr_Hz;  // freq error on PING (if available) else 0x7FFFFFFF
  uint16_t ping_len;        // PING length in bytes

  uint16_t crc; // CRC16-CCITT of all prior bytes
};

static const uint32_t ACK_MAGIC = 0x314B4341; // 'ACK1'

// -------------------- RAK4631 SX1262 wiring --------------------
// The RAK4631 datasheet lists the internal nRF52840 <-> SX1262 connections:
//   NSS=P1.10, SCK=P1.11, MOSI=P1.12, MISO=P1.13, RESET=P1.06, BUSY=P1.14, DIO1=P1.15
// (If your BSP maps these differently, adjust here or use the BSP's WB_* defines.)
//
// We only need NSS/DIO1/RST/BUSY for RadioLib's Module constructor.
// SX1262 wiring on RAK4631 (nRF52840)
#define PIN_LORA_NSS NRF_GPIO_PIN_MAP(1, 10)
#define PIN_LORA_SCK NRF_GPIO_PIN_MAP(1, 11)
#define PIN_LORA_MOSI NRF_GPIO_PIN_MAP(1, 12)
#define PIN_LORA_MISO NRF_GPIO_PIN_MAP(1, 13)

#define PIN_LORA_RST NRF_GPIO_PIN_MAP(1, 6)
#define PIN_LORA_ANT_SW NRF_GPIO_PIN_MAP(1, 5)
#define PIN_LORA_DIO1 NRF_GPIO_PIN_MAP(1, 15)
#define PIN_LORA_BUSY NRF_GPIO_PIN_MAP(1, 14)

SPIClass LoraSPI(NRF_SPIM1, PIN_LORA_MISO, PIN_LORA_SCK, PIN_LORA_MOSI);
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, LoraSPI);

static uint32_t receiverId()
{
#if defined(NRF_FICR) && defined(NRF_FICR_DEVICEID_DEVICEID_Msk)
  // nRF52840 has 2x 32-bit DEVICEID words.
  uint32_t a = NRF_FICR->DEVICEID[0];
  uint32_t b = NRF_FICR->DEVICEID[1];
  // quick hash/mix
  uint32_t x = a ^ (b * 0x9E3779B9u);
  x ^= (x >> 16);
  x *= 0x85EBCA6Bu;
  x ^= (x >> 13);
  x *= 0xC2B2AE35u;
  x ^= (x >> 16);
  return x;
#else
  // Fallback: not unique, but deterministic.
  return RECEIVERID;
#endif
}

static uint16_t computeDelayMs(uint32_t rid, uint16_t seq)
{
  // Static. Calculate based on Receiver ID * 100ms.
  return DELAY_MS;
}

static void buildAck(AckPacket &ack,
                     uint16_t seq,
                     uint16_t ping_len,
                     float rssi,
                     float snr,
                     int32_t freqErrHz,
                     uint32_t rid,
                     uint16_t delayMs)
{
  memset(&ack, 0, sizeof(ack));
  ack.magic = ACK_MAGIC;
  ack.version = 1;
  ack.seq = seq;

  ack.receiver_id = rid;
  ack.receiver_delay_ms = delayMs;
  ack.rx_uptime_s = millis() / 1000;

  // clamp to int16
  double r = isnan(rssi) ? 0.0 : (double)rssi;
  if (r > 32767)
    r = 32767;
  if (r < -32768)
    r = -32768;
  ack.ping_rssi_dBm_x1 = (int16_t)llround(r);

  double s10 = isnan(snr) ? 0.0 : (double)snr * 10.0;
  if (s10 > 32767)
    s10 = 32767;
  if (s10 < -32768)
    s10 = -32768;
  ack.ping_snr_dB_x10 = (int16_t)llround(s10);

  ack.ping_freqerr_Hz = freqErrHz;
  ack.ping_len = ping_len;

  ack.crc = 0;
  ack.crc = crc16_ccitt((const uint8_t *)&ack, sizeof(ack) - sizeof(ack.crc));
}

static volatile bool rxFlag = false;
static void setFlag(void)
{
  rxFlag = true;
}

void setup()
{
  Serial.begin(115200);
  delay(150);

  // Radio init
  // Power RF switch (important on many RAK boards)
  pinMode(PIN_LORA_ANT_SW, OUTPUT);
  digitalWrite(PIN_LORA_ANT_SW, HIGH);
  delay(10);
  LoraSPI.begin();

  int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("Radio init failed: ");
    Serial.println(state);
    while (true)
      delay(1000);
  }

  // Best-effort: match beacon's radio config-ish
  radio.setOutputPower(22);
  radio.setCRC(false); // we do our own CRC
  radio.setDio1Action(setFlag);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("startReceive failed: ");
    Serial.println(state);
  }

  Serial.println("RAK4631 receiver ready.");
  Serial.print("receiver_id=");
  Serial.println((unsigned long)receiverId());
  Serial.print("TelemetryPacket bytes=");
  Serial.println((unsigned)sizeof(TelemetryPacket));
  Serial.print("AckPacket bytes=");
  Serial.println((unsigned)sizeof(AckPacket));
}

void loop()
{
  if (!rxFlag)
    return;
  rxFlag = false;

  // Read RX
  uint8_t buf[256];
  int16_t len = radio.getPacketLength(true);
  if (len <= 0 || len > (int)sizeof(buf))
  {
    // restart RX
    radio.startReceive();
    return;
  }

  int state = radio.readData(buf, (size_t)len);
  float rssi = radio.getRSSI();
  float snr = radio.getSNR();
  int32_t fe = (int32_t)radio.getFrequencyError();

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("RX read failed: ");
    Serial.println(state);
    radio.startReceive();
    return;
  }

  // Quick magic check so we don't spam logs when we hear other traffic (e.g., other receivers' PONGs)
  uint32_t magic = 0;
  if (len >= 4)
  {
    memcpy(&magic, buf, sizeof(magic));
  }

  // If we heard an ACK/PONG (from another receiver, or our own due to close RF coupling), ignore it.
  if (magic == ACK_MAGIC)
  {
    radio.startReceive();
    return;
  }

  // If it isn't a telemetry ping, ignore silently.
  if (magic != TLMS_MAGIC)
  {
    radio.startReceive();
    return;
  }

  TelemetryPacket ping;
  if (!decodeTelemetry(buf, (size_t)len, ping))
  {
    Serial.print("RX invalid TLMS PING (len/crc). len=");
    Serial.print(len);
    Serial.print(" expected=");
    Serial.println((int)sizeof(TelemetryPacket));
    radio.startReceive();
    return;
  }

  const uint32_t rid = receiverId();
  const uint16_t dly = computeDelayMs(rid, ping.seq);

  Serial.print("Delayed PONG seq=");
  Serial.print((unsigned)ping.seq);
  Serial.print(" delay=");
  Serial.print((unsigned)dly);
  Serial.print("ms rssi=");
  Serial.print(rssi, 1);
  Serial.print(" snr=");
  Serial.println(snr, 1);
  Serial.println();

  // Backoff to reduce collisions among multiple receivers
  delay(dly);

  AckPacket ack;
  buildAck(ack, ping.seq, (uint16_t)len, rssi, snr, fe, rid, dly);

  state = radio.transmit((uint8_t *)&ack, sizeof(ack));
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.print("PONG sent seq=");
    Serial.print((unsigned)ping.seq);
    Serial.print(" delay=");
    Serial.print((unsigned)dly);
    Serial.print("ms rssi=");
    Serial.print(rssi, 1);
    Serial.print(" snr=");
    Serial.println(snr, 1);
  }
  else
  {
    Serial.print("PONG TX failed: ");
    Serial.println(state);
  }

  // Resume RX
  radio.startReceive();
}
