#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <math.h>

// -------------------- PINOUT (RX) --------------------
#define LORA_SCK   18
#define LORA_MISO  23
#define LORA_MOSI  19
#define LORA_CS    25
#define LORA_RST   14
#define LORA_DIO0  26

// -------------------- RADIO DEFAULT --------------------
static const uint32_t DEFAULT_FREQ_HZ = 868750000UL; // 868.750 MHz
static const int      DEFAULT_SF      = 10;          // egyezzen az adóval

static const long LORA_BW = 125E3;
static const int  LORA_CR_DENOM = 5; // 4/5

// -------------------- LEGACY REBUILD CONSTANTS --------------------
static const char* PACKET_TYPE = "TX1";
static const char* MISSION_ID  = "M1";
static const float SEA_LEVEL_HPA = 1013.25f;

// RH legyen NA vagy kamu fix átlagérték.
// Ha inkább fix átlagot szeretnél, állítsd false-ra, és az RH_FAKE_VALUE kerül ki.
static const bool  RH_OUTPUT_NA   = false;
static const float RH_FAKE_VALUE  = 50.0f;

// Akku százalék számításhoz
static const float BAT_V_EMPTY = 3.30f;
static const float BAT_V_FULL  = 4.20f;

// -------------------- GLOBALS --------------------
BluetoothSerial SerialBT;
Preferences prefs;

static bool     btOk       = false;
static uint32_t loraFreqHz = DEFAULT_FREQ_HZ;
static int      loraSf     = DEFAULT_SF;

// SEQ tracking
static bool     hasLastSeq = false;
static uint32_t lastSeq    = 0;

// -------------------- forward declarations --------------------
void stableBootIntro();
bool waitForUserToEnterConfig(uint32_t windowMs);
void configMenuAtBoot();
void printCurrentRadioSettings();

static inline bool isValidFreq(uint32_t f);
static inline bool isValidSf(int sf);

uint32_t loadFreqFromNVS();
void saveFreqToNVS(uint32_t f);
int loadSfFromNVS();
void saveSfToNVS(int sf);

String readLineWithTimeout(uint32_t timeoutMs);
String readLineBlocking();
static inline void outLine(const String& s);

static int countChar(const String& s, char c);
static bool getCsvField(const String& payload, int fieldIndex, String& out);
static bool isIntegerToken(const String& s);
static bool isNumericToken(const String& s);

static uint8_t batteryPercentFromVoltage(float vbat);
static float altitudeFromPressure(float pressure_hPa);

static bool tryParseShortSeq(const String& payload, uint32_t& seqOut);
void checkSeqJumpAndReport_Short(const String& payload);

bool rebuildShortPacketToLegacyCsv(const String& payload, int packetRssi, float packetSnr, String& legacyOut);

void setupBluetooth();
void setupLoRa();

// -------------------- Helpers --------------------
static inline bool isValidFreq(uint32_t f) { return (f >= 863000000UL && f <= 870000000UL); }
static inline bool isValidSf(int sf)       { return (sf >= 8 && sf <= 11); }

static int countChar(const String& s, char c)
{
  int n = 0;
  for (size_t i = 0; i < s.length(); i++) {
    if (s[i] == c) n++;
  }
  return n;
}

uint32_t loadFreqFromNVS()
{
  prefs.begin("slinky", true);
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

int loadSfFromNVS()
{
  prefs.begin("slinky", true);
  int sf = prefs.getInt("sf", DEFAULT_SF);
  prefs.end();
  if (!isValidSf(sf)) sf = DEFAULT_SF;
  return sf;
}

void saveSfToNVS(int sf)
{
  prefs.begin("slinky", false);
  prefs.putInt("sf", sf);
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

      if (c == 8 || c == 127) {
        if (s.length()) s.remove(s.length() - 1);
      } else {
        s += c;
      }
    }
    delay(5);
  }

  return "";
}

String readLineBlocking()
{
  String s;

  while (true) {
    while (Serial.available()) {
      char c = (char)Serial.read();

      if (c == '\r') continue;

      if (c == '\n') {
        if (s.length() > 0) return s;
        // üres ENTER esetén is térjünk vissza üres stringgel
        return "";
      }

      if (c == 8 || c == 127) {
        if (s.length()) s.remove(s.length() - 1);
      } else {
        s += c;
      }
    }
    delay(5);
  }
}

static inline void outLine(const String& s)
{
  Serial.println(s);
  if (btOk && SerialBT.hasClient()) {
    SerialBT.println(s);
  }
}

static bool getCsvField(const String& payload, int fieldIndex, String& out)
{
  int start = 0;
  int idx = 0;

  while (true) {
    int comma = payload.indexOf(',', start);

    if (comma < 0) {
      if (idx == fieldIndex) {
        out = payload.substring(start);
        out.trim();
        return true;
      }
      return false;
    }

    if (idx == fieldIndex) {
      out = payload.substring(start, comma);
      out.trim();
      return true;
    }

    idx++;
    start = comma + 1;
    if (start > (int)payload.length()) return false;
  }
}

static bool isIntegerToken(const String& s)
{
  if (s.length() == 0) return false;

  size_t i = 0;
  if (s[0] == '-' || s[0] == '+') {
    if (s.length() == 1) return false;
    i = 1;
  }

  for (; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

static bool isNumericToken(const String& s)
{
  if (s.length() == 0) return false;
  if (s == "NA" || s == "---") return false;

  bool hasDigit = false;
  bool hasDot   = false;

  size_t i = 0;
  if (s[0] == '-' || s[0] == '+') {
    if (s.length() == 1) return false;
    i = 1;
  }

  for (; i < s.length(); i++) {
    char c = s[i];
    if (isDigit(c)) {
      hasDigit = true;
    } else if (c == '.') {
      if (hasDot) return false;
      hasDot = true;
    } else {
      return false;
    }
  }

  return hasDigit;
}

static uint8_t batteryPercentFromVoltage(float vbat)
{
  float pct = (vbat - BAT_V_EMPTY) / (BAT_V_FULL - BAT_V_EMPTY) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (uint8_t)lroundf(pct);
}

static float altitudeFromPressure(float pressure_hPa)
{
  if (!isfinite(pressure_hPa)) return NAN;
  if (pressure_hPa < 100.0f || pressure_hPa > 1100.0f) return NAN;

  float ratio = pressure_hPa / SEA_LEVEL_HPA;
  return 44330.0f * (1.0f - powf(ratio, 0.1903f));
}

// -------------------- Stable boot UX --------------------
void stableBootIntro()
{
  Serial.println();
  Serial.println("=======================================");
  Serial.println("   SlinkyReceiver - Legacy Rebuilder");
  Serial.println("=======================================");
  Serial.println("Waiting 3 seconds for USB serial...");
  delay(3000);
  Serial.println("Serial ready.");
}

bool waitForUserToEnterConfig(uint32_t windowMs)
{
  Serial.println();
  Serial.print("Press ENTER within ");
  Serial.print((unsigned long)(windowMs / 1000));
  Serial.println("s to configure radio...");

  uint32_t start = millis();
  while (millis() - start < windowMs) {
    if (Serial.available()) {
      while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
          Serial.println("Entering radio configuration...");
          return true;
        }
      }
    }
    delay(10);
  }

  Serial.println("Skipping configuration.");
  return false;
}

void printCurrentRadioSettings()
{
  Serial.print("Current LoRa freq (Hz): ");
  Serial.println(loraFreqHz);
  Serial.print("Current LoRa SF (8..11): ");
  Serial.println(loraSf);
}

void configMenuAtBoot()
{
  loraFreqHz = loadFreqFromNVS();
  loraSf     = loadSfFromNVS();

  Serial.println();
  Serial.println("=== CONFIGURATION MENU ===");

  while (true) {
    Serial.println();
    Serial.println("1 = Set frequency");
    Serial.println("2 = Set Spreading Factor (SF)");
    Serial.println("3 = Show current settings");
    Serial.println("0 = Save and start receiver");
    Serial.print("Choice: ");

    String cmd = readLineBlocking();
    cmd.trim();

    if (cmd == "1") {
      Serial.println();
      Serial.print("Enter frequency in Hz: ");
      String line = readLineBlocking();
      line.trim();

      if (line.length() == 0) {
        Serial.println("Empty input -> keeping current frequency.");
        continue;
      }

      uint32_t newF = (uint32_t)strtoul(line.c_str(), nullptr, 10);
      if (isValidFreq(newF)) {
        loraFreqHz = newF;
        Serial.print("New frequency staged: ");
        Serial.println(loraFreqHz);
      } else {
        Serial.println("Invalid frequency! Valid range: 863000000..870000000");
      }
    }
    else if (cmd == "2") {
      Serial.println();
      Serial.print("Enter SF (8..11): ");
      String line = readLineBlocking();
      line.trim();

      if (line.length() == 0) {
        Serial.println("Empty input -> keeping current SF.");
        continue;
      }

      int newSf = (int)strtol(line.c_str(), nullptr, 10);
      if (isValidSf(newSf)) {
        loraSf = newSf;
        Serial.print("New SF staged: ");
        Serial.println(loraSf);
      } else {
        Serial.println("Invalid SF! Valid range: 8..11");
      }
    }
    else if (cmd == "3") {
      Serial.println();
      printCurrentRadioSettings();
    }
    else if (cmd == "0") {
      saveFreqToNVS(loraFreqHz);
      saveSfToNVS(loraSf);

      Serial.println();
      Serial.println("Settings saved.");
      printCurrentRadioSettings();
      Serial.println("Starting receiver...");
      return;
    }
    else {
      Serial.println("Unknown command.");
    }
  }
}

// -------------------- Short packet SEQ parsing --------------------
// Short TX format:
// T,SEQ,TIN,TOUT,P,UBAT,LATi,LONi,ALTi
// field[1] = SEQ
static bool tryParseShortSeq(const String& payload, uint32_t& seqOut)
{
  String f0, f1;

  if (!getCsvField(payload, 0, f0)) return false;
  if (!getCsvField(payload, 1, f1)) return false;

  if (f0 != "T") return false;
  if (!isIntegerToken(f1)) return false;

  seqOut = (uint32_t)strtoul(f1.c_str(), nullptr, 10);
  return true;
}

void checkSeqJumpAndReport_Short(const String& payload)
{
  uint32_t seqNow = 0;
  if (!tryParseShortSeq(payload, seqNow)) return;

  if (!hasLastSeq) {
    hasLastSeq = true;
    lastSeq = seqNow;
    return;
  }

  if (seqNow == lastSeq + 1) {
    lastSeq = seqNow;
    return;
  }

  if (seqNow <= lastSeq) {
    outLine(String("!! SEQ reset/backjump: last=") + lastSeq + " now=" + seqNow);
    lastSeq = seqNow;
    return;
  }

  uint32_t missed = (seqNow - lastSeq) - 1;
  outLine(String("!! SEQ jump: missed=") + missed + " (last=" + lastSeq + " now=" + seqNow + ")");
  lastSeq = seqNow;
}

// -------------------- Short -> Legacy CSV rebuild --------------------
bool rebuildShortPacketToLegacyCsv(const String& payload, int packetRssi, float packetSnr, String& legacyOut)
{
  // Expected:
  // 0:T
  // 1:SEQ
  // 2:T_in_C
  // 3:T_out_C
  // 4:P_hPa
  // 5:BAT_V
  // 6:LAT_i
  // 7:LON_i
  // 8:ALT_gps_m

  if (countChar(payload, ',') != 8) return false;

  String f0, f1, f2, f3, f4, f5, f6, f7, f8;
  if (!getCsvField(payload, 0, f0)) return false;
  if (!getCsvField(payload, 1, f1)) return false;
  if (!getCsvField(payload, 2, f2)) return false;
  if (!getCsvField(payload, 3, f3)) return false;
  if (!getCsvField(payload, 4, f4)) return false;
  if (!getCsvField(payload, 5, f5)) return false;
  if (!getCsvField(payload, 6, f6)) return false;
  if (!getCsvField(payload, 7, f7)) return false;
  if (!getCsvField(payload, 8, f8)) return false;

  if (f0 != "T") return false;
  if (!isIntegerToken(f1)) return false;

  uint32_t seq = (uint32_t)strtoul(f1.c_str(), nullptr, 10);
  uint32_t msNow = millis();

  // UTC nincs a rövid csomagban
  String utcStr = "19700101-000000";

  String tinStr = "NA";
  if (isNumericToken(f2)) {
    float tin = f2.toFloat();
    if (isfinite(tin) && tin > -80.0f && tin < 120.0f) {
      tinStr = String(tin, 1);
    }
  }

  String toutStr = "NA";
  if (isNumericToken(f3)) {
    float tout = f3.toFloat();
    if (isfinite(tout) && tout > -80.0f && tout < 120.0f) {
      toutStr = String(tout, 1);
    }
  }

  String pStr = "NA";
  float pressure = NAN;
  if (isNumericToken(f4)) {
    pressure = f4.toFloat();
    if (isfinite(pressure) && pressure > 100.0f && pressure < 1100.0f) {
      pStr = String(pressure, 1);
    } else {
      pressure = NAN;
    }
  }

  String rhStr = "NA";
  if (!RH_OUTPUT_NA) {
    rhStr = String(RH_FAKE_VALUE, 1);
  }

  String hBmeStr = "NA";
  if (isfinite(pressure)) {
    float h = altitudeFromPressure(pressure);
    if (isfinite(h) && h > -1000.0f && h < 50000.0f) {
      hBmeStr = String(h, 1);
    }
  }

  String batVStr = "NA";
  String batPctStr = "NA";
  if (isNumericToken(f5)) {
    float batV = f5.toFloat();
    if (isfinite(batV) && batV > 0.5f && batV < 6.5f) {
      batVStr = String(batV, 2);
      batPctStr = String((int)batteryPercentFromVoltage(batV));
    }
  }

  String latStr = "NA";
  String lonStr = "NA";
  String altGpsStr = "NA";

  if (isIntegerToken(f6) && isIntegerToken(f7) && isIntegerToken(f8)) {
    long lat_i = strtol(f6.c_str(), nullptr, 10);
    long lon_i = strtol(f7.c_str(), nullptr, 10);
    long alt_i = strtol(f8.c_str(), nullptr, 10);

    float lat = lat_i / 10000.0f;
    float lon = lon_i / 10000.0f;
    float alt = (float)alt_i;

    if (lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f) {
      latStr = String(lat, 4);
      lonStr = String(lon, 4);
      altGpsStr = String(alt, 1);
    }
  }

  String rssiStr = String(packetRssi);
  String snrStr  = String(packetSnr, 1);

  legacyOut.reserve(192);
  legacyOut = "";
  legacyOut += PACKET_TYPE; legacyOut += ",";
  legacyOut += MISSION_ID;  legacyOut += ",";
  legacyOut += String(seq); legacyOut += ",";
  legacyOut += String(msNow); legacyOut += ",";
  legacyOut += utcStr;      legacyOut += ",";
  legacyOut += pStr;        legacyOut += ",";
  legacyOut += tinStr;      legacyOut += ",";
  legacyOut += rhStr;       legacyOut += ",";
  legacyOut += toutStr;     legacyOut += ",";
  legacyOut += hBmeStr;     legacyOut += ",";
  legacyOut += latStr;      legacyOut += ",";
  legacyOut += lonStr;      legacyOut += ",";
  legacyOut += altGpsStr;   legacyOut += ",";
  legacyOut += batVStr;     legacyOut += ",";
  legacyOut += batPctStr;   legacyOut += ",";
  legacyOut += rssiStr;     legacyOut += ",";
  legacyOut += snrStr;

  return true;
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
  Serial.print(" Hz, SF=");
  Serial.print(loraSf);
  Serial.print("... ");

  if (!LoRa.begin((long)loraFreqHz)) {
    Serial.println("FAILED");
    while (true) delay(1000);
  }
  Serial.println("OK");

  LoRa.enableCrc();
  LoRa.setSpreadingFactor(loraSf);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR_DENOM);
}

// -------------------- Arduino --------------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  stableBootIntro();

  if (waitForUserToEnterConfig(5000)) {
    configMenuAtBoot();
  } else {
    loraFreqHz = loadFreqFromNVS();
    loraSf     = loadSfFromNVS();
  }

  Serial.println("=== SlinkyReceiver: short packet -> legacy CSV + RSSI/SNR ===");
  outLine("Output CSV: PKT,MISSION,SEQ,MS,UTC,P_hPa,T_in_C,RH_pct,T_out_C,H_bme_m,LAT,LON,ALT_gps_m,BAT_V,BAT_PCT,RSSI,SNR");

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
  while (LoRa.available()) {
    payload += (char)LoRa.read();
  }
  payload.trim();

  int   packetRssi = LoRa.packetRssi();
  float packetSnr  = LoRa.packetSnr();

  // Rövid csomag: T,seq,tin,tout,p,ubat,lat_i,lon_i,alt_i
  if (countChar(payload, ',') == 8) {
    checkSeqJumpAndReport_Short(payload);

    String legacyLine;
    if (rebuildShortPacketToLegacyCsv(payload, packetRssi, packetSnr, legacyLine)) {
      outLine(legacyLine);
    } else {
      outLine(String("!! WARN: short packet parse failed: ") + payload);
    }
    return;
  }

  // Ha mégis régi 15 mezős csomag jönne, hozzáfűzzük az RSSI/SNR mezőket
  if (countChar(payload, ',') == 14) {
    String extended = payload + "," + String(packetRssi) + "," + String(packetSnr, 1);
    outLine(extended);
    return;
  }

  outLine(String("!! WARN: unexpected packet format (commas=") + countChar(payload, ',') + "): " + payload);
}