#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "BluetoothSerial.h"
#include <Preferences.h>

// -------------------- PINOUT (RX) --------------------
#define LORA_SCK   18
#define LORA_MISO  23
#define LORA_MOSI  19
#define LORA_CS    25
#define LORA_RST   14
#define LORA_DIO0  26

// -------------------- RADIO DEFAULT --------------------
static const uint32_t DEFAULT_FREQ_HZ = 868750000UL; // 868.750 MHz

// LoRa paraméterek (egyezzenek az adóval)
static const int  LORA_SF = 9;
static const long LORA_BW = 125E3;
static const int  LORA_CR_DENOM = 5; // 4/5

BluetoothSerial SerialBT;
Preferences prefs;

static bool btOk = false;
static uint32_t loraFreqHz = DEFAULT_FREQ_HZ;

// SEQ tracking
static bool hasLastSeq = false;
static uint32_t lastSeq = 0;

// -------------------- Helpers --------------------
static inline bool isValidFreq(uint32_t f)
{
  return (f >= 863000000UL && f <= 870000000UL);
}

uint32_t loadFreqFromNVS()
{
  prefs.begin("slinky", true); // read-only
  uint32_t f = prefs.getUInt("freq_hz", DEFAULT_FREQ_HZ);
  prefs.end();
  if (!isValidFreq(f)) f = DEFAULT_FREQ_HZ;
  return f;
}

void saveFreqToNVS(uint32_t f)
{
  prefs.begin("slinky", false);
  prefs.putUInt("freq_hz", f);
  prefs.end();
}

String readLineWithTimeout(uint32_t timeoutMs)
{
  String s;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') return s;
      if (c == 8 || c == 127) { // backspace
        if (s.length()) s.remove(s.length() - 1);
      } else {
        s += c;
      }
    }
    delay(5);
  }
  return ""; // timeout
}

void maybeChangeFrequencyAtBoot()
{
  loraFreqHz = loadFreqFromNVS();

  Serial.println();
  Serial.println("=== SlinkyReceiver boot ===");
  Serial.print("Saved LoRa freq (Hz): ");
  Serial.println(loraFreqHz);

  Serial.println("Enter new frequency in Hz within 10s (or press Enter to keep):");
  Serial.print("> ");

  String line = readLineWithTimeout(10000);
  line.trim();

  if (line.length() == 0) {
    Serial.println();
    Serial.println("No input -> keeping saved frequency.");
    return;
  }

  uint32_t newF = (uint32_t) strtoul(line.c_str(), nullptr, 10);
  if (!isValidFreq(newF)) {
    Serial.println();
    Serial.println("Invalid frequency! Keeping saved frequency.");
    return;
  }

  loraFreqHz = newF;
  saveFreqToNVS(loraFreqHz);

  Serial.println();
  Serial.print("New frequency saved: ");
  Serial.println(loraFreqHz);
}

static inline void outLine(const String& s)
{
  Serial.println(s);
  if (btOk && SerialBT.hasClient()) {
    SerialBT.println(s);
  }
}

// --- SEQ parsing: supports "SEQ=123," and "123,...." ---
bool tryParseSeq(const String& payload, uint32_t& seqOut)
{
  // Case 1: starts with digits then comma -> CSV where first field is seq
  if (payload.length() >= 2 && isDigit(payload[0])) {
    int comma = payload.indexOf(',');
    if (comma > 0) {
      String first = payload.substring(0, comma);
      first.trim();
      // ensure all digits
      for (size_t i = 0; i < first.length(); i++) {
        if (!isDigit(first[i])) { comma = -1; break; }
      }
      if (comma > 0) {
        seqOut = (uint32_t) strtoul(first.c_str(), nullptr, 10);
        return true;
      }
    }
  }

  // Case 2: contains "SEQ="
  int p = payload.indexOf("SEQ=");
  if (p >= 0) {
    int start = p + 4;
    int end = start;
    while (end < (int)payload.length() && isDigit(payload[end])) end++;
    if (end > start) {
      String num = payload.substring(start, end);
      seqOut = (uint32_t) strtoul(num.c_str(), nullptr, 10);
      return true;
    }
  }

  return false;
}

void checkSeqJumpAndReport(const String& payload)
{
  uint32_t seqNow = 0;
  if (!tryParseSeq(payload, seqNow)) return;

  if (!hasLastSeq) {
    hasLastSeq = true;
    lastSeq = seqNow;
    return;
  }

  // normál eset: +1
  if (seqNow == lastSeq + 1) {
    lastSeq = seqNow;
    return;
  }

  // újraindulás / wrap / visszaugrás
  if (seqNow <= lastSeq) {
    String warn = String("!! SEQ reset/backjump: last=") + lastSeq + " now=" + seqNow;
    outLine(warn);
    lastSeq = seqNow;
    return;
  }

  // ugrás előre -> csomagvesztés
  uint32_t missed = (seqNow - lastSeq) - 1;
  String warn = String("!! SEQ jump: missed=") + missed + " (last=" + lastSeq + " now=" + seqNow + ")";
  outLine(warn);

  lastSeq = seqNow;
}

// -------------------- Setup parts --------------------
void setupBluetooth()
{
  Serial.print("Bluetooth SPP init... ");
  btOk = SerialBT.begin("slinkyreceiver");
  Serial.println(btOk ? "OK" : "FAILED");
}

void setupLoRa()
{
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  Serial.print("LoRa init @ ");
  Serial.print(loraFreqHz);
  Serial.print(" Hz... ");

  if (!LoRa.begin((long)loraFreqHz)) {
    Serial.println("FAILED");
    while (true) delay(1000);
  }
  Serial.println("OK");

  LoRa.enableCrc();
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR_DENOM);
}

// -------------------- Arduino --------------------
void setup()
{
  Serial.begin(115200);
  delay(300);

  maybeChangeFrequencyAtBoot();

  Serial.println("=== SlinkyReceiver: LoRa RX + BT SPP ===");
  setupBluetooth();
  setupLoRa();

  outLine("Receiver ready.");
}

void loop()
{
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String payload;
  payload.reserve(packetSize + 8);
  while (LoRa.available()) payload += (char)LoRa.read();

  // SEQ ugrás figyelése (ha felismeri a formátumot)
  checkSeqJumpAndReport(payload);

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  String line = payload;
  line += " | RSSI=" + String(rssi) + " dBm";
  line += " | SNR=" + String(snr, 1);

  outLine(line);
}
