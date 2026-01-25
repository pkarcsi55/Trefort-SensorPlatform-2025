/*
  ESP32 (Wemos Lolin32 Lite) - Demo for CanSat system
  Serial commands (USB or Bluetooth SPP):
    A -> one-shot full hardware check (SD, BME, HX711, Battery, Hall, GPS) (LoRa quick status)
    B -> BME: Pressure, Temperature, Humidity, Altitude
    F -> Force (HX711) one-shot
    H -> Hall one-shot
    G -> GPS status (UTC, location, etc.)
    P -> ~50 Hz stream: force,hall (CSV for plotter)
    L -> LoRa SPI alive test (SX127x RegVersion)
    T -> LoRa TX test packet (1 packet)
    ? / M -> HELP menu (anytime)

  Bluetooth:
    Device name: "SlinkySpaceDemo"
    USB always works. Bluetooth output only if a client is connected (hasClient()).

  Pinout from project:
    SD CS   = GPIO5
    SD SCK  = GPIO18
    SD MISO = GPIO23
    SD MOSI = GPIO19

  Peripherals:
    BME280 I2C: SDA=GPIO32, SCL=GPIO33
    HX711: DOUT=GPIO22, SCK=GPIO27
    Hall analog: GPIO34  (3.3V)
    Battery divider analog: GPIO35
    GPS UART2: RX=GPIO16, TX=GPIO17

  LoRa (RFM95 / SX1276) (same as receiver):
    SCK  18
    MISO 23
    MOSI 19
    CS   25
    RST  14
    DIO0 26
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <HX711.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#include <LoRa.h>
#include "BluetoothSerial.h"

// ---------------- Bluetooth ----------------
BluetoothSerial SerialBT;
static const char* BT_NAME = "SlinkySpaceDemo";
bool btInitOk = false;   // avoids collision with esp32-hal btStarted()

// ---------------- Pin mapping ----------------
static const int PIN_HALL_ADC     = 34;
static const int PIN_BATT_ADC     = 35;

static const float VBAT_CAL = 1.066f;

static const int PIN_I2C_SDA      = 32;
static const int PIN_I2C_SCL      = 33;

static const int PIN_HX711_DOUT   = 22;
static const int PIN_HX711_SCK    = 27;

static const int PIN_SD_CS        = 5;
static const int PIN_SD_SCK       = 18;
static const int PIN_SD_MISO      = 23;
static const int PIN_SD_MOSI      = 19;

// GPS UART2 mapping
static const int PIN_GPS_RX2      = 16;
static const int PIN_GPS_TX2      = 17;

// LoRa (RFM95/SX1276) mapping
static const int PIN_LORA_CS      = 25;
static const int PIN_LORA_RST     = 14;
static const int PIN_LORA_DIO0    = 26;

// ---------------- Objects ----------------
Adafruit_BME280 bme;
bool bmeOk = false;

HX711 scale;
bool hxOk = false;

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// Use VSPI explicitly for SD + LoRa
SPIClass vspi(VSPI);
bool sdOk = false;

// LoRa status
bool loraOk = false;

// ---------------- Constants ----------------
static const float SEA_LEVEL_HPA = 1013.25f;

// ADC
static const int   ADC_BITS      = 12;
static const float ADC_REF_V     = 3.30f;
static const float ADC_MAX       = 4095.0f;

// Battery divider 100k / 100k
static const float R1_OHMS = 100000.0f;
static const float R2_OHMS = 100000.0f;
static const float VBAT_MIN = 3.30f;
static const float VBAT_MAX = 4.20f;

// ---------------- Modes ----------------
enum Mode { MODE_IDLE = 0, MODE_PLOT_50HZ };
Mode mode = MODE_IDLE;

// plot timing
static const uint32_t PLOT_PERIOD_MS = 20; // 50 Hz ~ 20 ms
uint32_t lastPlotMs = 0;

// Hall zero
float hallZeroRaw = NAN;

// GPS bookkeeping
static const uint32_t GPS_BAUD = 9600;
uint32_t gpsCharsTotal = 0;
uint32_t gpsLastCharMs = 0;
bool gpsEverHadChars = false;

// ---------------- Universal IO (USB always, BT only if connected) ----------------
inline bool btConnected() {
  return btInitOk && SerialBT.hasClient();
}

void uPrint(const String& s) { Serial.print(s); if (btConnected()) SerialBT.print(s); }
void uPrintln(const String& s = "") { Serial.println(s); if (btConnected()) SerialBT.println(s); }

void uPrint(const char* s) { Serial.print(s); if (btConnected()) SerialBT.print(s); }
void uPrintln(const char* s) { Serial.println(s); if (btConnected()) SerialBT.println(s); }

void uPrint(char c) { Serial.print(c); if (btConnected()) SerialBT.print(c); }
void uPrint(long v) { Serial.print(v); if (btConnected()) SerialBT.print(v); }
void uPrint(int v)  { Serial.print(v); if (btConnected()) SerialBT.print(v); }

void uPrintln(long v) { Serial.println(v); if (btConnected()) SerialBT.println(v); }
void uPrintln(int v)  { Serial.println(v); if (btConnected()) SerialBT.println(v); }

bool readCommandChar(char &c) {
  if (Serial.available()) { c = (char)Serial.read(); return true; }
  if (btConnected() && SerialBT.available()) { c = (char)SerialBT.read(); return true; }
  return false;
}

// ---------------- Helpers ----------------
float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

int estimateLiIonPercent(float vbat) {
  float p = (vbat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0f;
  p = clampf(p, 0.0f, 100.0f);
  return (int)(p + 0.5f);
}

float readAdcAveraged(int pin, int samples) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return (float)sum / (float)samples;
}

void print2digits(int v) {
  if (v < 10) uPrint('0');
  uPrint(v);
}

void gpsPump(unsigned long maxMs = 10) {
  unsigned long start = millis();
  while (millis() - start < maxMs) {
    while (GPSSerial.available()) {
      char c = (char)GPSSerial.read();
      gps.encode(c);

      gpsCharsTotal++;
      gpsLastCharMs = millis();
      gpsEverHadChars = true;
    }
  }
}

// ---------------- SD ----------------
bool sdWriteTest(const char* path, const String& text) {
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  size_t n = f.println(text);
  f.close();
  return (n > 0);
}

String sdReadAll(const char* path) {
  File f = SD.open(path, FILE_READ);
  if (!f) return String("");
  String content;
  while (f.available()) content += char(f.read());
  f.close();
  return content;
}

void listDirRootBrief() {
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    uPrintln("SD: root open FAILED");
    return;
  }
  uPrintln("SD: Listing / (brief)");
  File entry;
  int count = 0;
  while ((entry = root.openNextFile())) {
    uPrint("  ");
    uPrint(entry.name());
    if (entry.isDirectory()) uPrintln("/");
    else {
      uPrint(" (");
      uPrint((long)entry.size());
      uPrintln(" B)");
    }
    entry.close();
    if (++count >= 8) {
      uPrintln("  ...");
      break;
    }
  }
  root.close();
}

bool initSD() {
  // VSPI bus pins
  vspi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  return SD.begin(PIN_SD_CS, vspi, 20000000);
}

// ---------------- BME280 ----------------
bool initBME() {
  // Try both common addresses
  if (bme.begin(0x76, &Wire)) return true;
  if (bme.begin(0x77, &Wire)) return true;
  return false;
}

void setupBME() {
  // Similar “normal” sampling
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X2,   // temperature
    Adafruit_BME280::SAMPLING_X16,  // pressure
    Adafruit_BME280::SAMPLING_X1,   // humidity
    Adafruit_BME280::FILTER_X16,
    Adafruit_BME280::STANDBY_MS_125
  );
  delay(50);
}

void printBmeOnce() {
  if (!bmeOk) {
    uPrintln("BME: NOT OK (init failed)");
    return;
  }

  float t = bme.readTemperature();
  float p_hPa = bme.readPressure() / 100.0f;
  float rh = bme.readHumidity();
  float alt = bme.readAltitude(SEA_LEVEL_HPA);

  uPrint("BME: T=");
  Serial.print(t, 2); if (btConnected()) SerialBT.print(t, 2);
  uPrint(" C, P=");
  Serial.print(p_hPa, 2); if (btConnected()) SerialBT.print(p_hPa, 2);
  uPrint(" hPa, RH=");
  Serial.print(rh, 1); if (btConnected()) SerialBT.print(rh, 1);
  uPrint(" %, ALT=");
  Serial.print(alt, 1); if (btConnected()) SerialBT.print(alt, 1);
  uPrintln(" m");
}

// ---------------- HX711 ----------------
bool initHX711() {
  scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  if (!scale.wait_ready_timeout(1500)) return false;
  delay(800);
  scale.tare();
  return true;
}

long hxReadRaw() {
  if (!hxOk) return LONG_MIN;
  if (!scale.wait_ready_timeout(100)) return LONG_MIN;
  return scale.read();
}

long hxReadZeroed() {
  if (!hxOk) return LONG_MIN;
  if (!scale.wait_ready_timeout(100)) return LONG_MIN;
  return scale.get_value(1);
}

void printForceOnce() {
  if (!hxOk) {
    uPrintln("HX711: NOT OK (no ready)");
    return;
  }
  long raw = hxReadRaw();
  long zeroed = hxReadZeroed();

  uPrint("FORCE(HX711): RAW=");
  uPrint((long)(raw == LONG_MIN ? 0 : raw));

  uPrint("  ZEROED=");
  uPrint((long)(zeroed == LONG_MIN ? 0 : zeroed));

  uPrintln();
}

// ---------------- Battery + Hall ----------------
void printBatteryOnce() {
  float adcRaw = readAdcAveraged(PIN_BATT_ADC, 40);
  float v_adc = (adcRaw / ADC_MAX) * ADC_REF_V;

  float divider = (R1_OHMS + R2_OHMS) / R2_OHMS; // 2
  float v_bat = v_adc * divider * VBAT_CAL;
  int pct = estimateLiIonPercent(v_bat);

  uPrint("BATT: ADCraw=");
  Serial.print(adcRaw, 1); if (btConnected()) SerialBT.print(adcRaw, 1);
  uPrint("  Vadc=");
  Serial.print(v_adc, 3); if (btConnected()) SerialBT.print(v_adc, 3);
  uPrint(" V  Vbat=");
  Serial.print(v_bat, 3); if (btConnected()) SerialBT.print(v_bat, 3);
  uPrint(" V  ~");
  uPrint(pct);
  uPrintln("%");
}

float readHallRaw(int samples = 40) {
  return readAdcAveraged(PIN_HALL_ADC, samples);
}

void printHallOnce() {
  float raw = readHallRaw(40);
  float v = (raw / ADC_MAX) * ADC_REF_V;
  if (isnan(hallZeroRaw)) hallZeroRaw = raw;
  float d = raw - hallZeroRaw;

  uPrint("HALL: ADCraw=");
  Serial.print(raw, 1); if (btConnected()) SerialBT.print(raw, 1);
  uPrint("  V=");
  Serial.print(v, 3); if (btConnected()) SerialBT.print(v, 3);
  uPrint(" V  dRAW=");
  Serial.print(d, 1); if (btConnected()) SerialBT.print(d, 1);
  uPrintln();
}

// ---------------- GPS ----------------
void printGpsStatus() {
  uPrintln("GPS: --- status ---");

  if (!gpsEverHadChars) {
    uPrintln("GPS: no serial chars yet. (check wiring: GPS TX -> GPIO16, baud 9600, GND common)");
    return;
  }

  uPrint("GPS: chars=");
  uPrint((long)gpsCharsTotal);
  uPrint("  lastCharAgo(ms)=");
  uPrintln((long)(millis() - gpsLastCharMs));

  uPrint("GPS: checksum ok sentences=");
  uPrintln((long)gps.passedChecksum());

  uPrint("GPS: satellites=");
  if (gps.satellites.isValid()) uPrintln((int)gps.satellites.value());
  else uPrintln("n/a");

  uPrint("GPS: HDOP=");
  if (gps.hdop.isValid()) { Serial.println(gps.hdop.hdop(), 1); if (btConnected()) SerialBT.println(gps.hdop.hdop(), 1); }
  else uPrintln("n/a");

  if (gps.location.isValid()) {
    uPrint("GPS: lat=");
    Serial.print(gps.location.lat(), 6); if (btConnected()) SerialBT.print(gps.location.lat(), 6);
    uPrint(" lon=");
    Serial.println(gps.location.lng(), 6); if (btConnected()) SerialBT.println(gps.location.lng(), 6);
  } else {
    uPrintln("GPS: location=n/a (no fix yet)");
  }

  uPrint("GPS: alt(m)=");
  if (gps.altitude.isValid()) { Serial.println(gps.altitude.meters(), 1); if (btConnected()) SerialBT.println(gps.altitude.meters(), 1); }
  else uPrintln("n/a");

  if (gps.date.isValid() && gps.time.isValid()) {
    uPrint("GPS: UTC ");
    uPrint((int)gps.date.year()); uPrint("-");
    print2digits(gps.date.month()); uPrint("-");
    print2digits(gps.date.day()); uPrint(" ");
    print2digits(gps.time.hour()); uPrint(":");
    print2digits(gps.time.minute()); uPrint(":");
    print2digits(gps.time.second());
    uPrintln();
  } else {
    uPrintln("GPS: date/time=n/a");
  }
}

// ---------------- LoRa (RFM95/SX1276) ----------------
#define REG_VERSION 0x42

uint8_t sx127xReadReg(uint8_t addr) {
  digitalWrite(PIN_LORA_CS, LOW);
  vspi.transfer(addr & 0x7F);
  uint8_t v = vspi.transfer(0x00);
  digitalWrite(PIN_LORA_CS, HIGH);
  return v;
}

bool initLoRaTX() {
  // Use same VSPI as SD (pins already set by vspi.begin)
  LoRa.setSPI(vspi);
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(868E6)) return false;

  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  return true;
}

void loraSpiAliveTest() {
  uPrintln("LoRa SPI alive test:");
  uint8_t ver = sx127xReadReg(REG_VERSION);

  uPrint("  RegVersion (0x42) = 0x");
  Serial.print(ver, HEX); if (btConnected()) SerialBT.print(ver, HEX);

  if (ver == 0x12) uPrintln("  OK (SX1276/78)");
  else if (ver == 0x00 || ver == 0xFF) uPrintln("  ERROR (SPI/CS/power?)");
  else uPrintln("  WARN (responding, unusual)");
}

void loraSendTest() {
  if (!loraOk) {
    uPrintln("LoRa TX: NOT OK (init failed)");
    return;
  }

  String msg = "CANSAT_TX_TEST," + String(millis());

  LoRa.beginPacket();
  LoRa.print(msg);
  int ok = LoRa.endPacket(true); // async

  uPrint("LoRa TX sent: ");
  uPrint(msg);
  uPrint("  endPacket=");
  uPrintln(ok);
}

// ---------------- HELP / UI ----------------
const char* modeName() {
  return (mode == MODE_PLOT_50HZ) ? "PLOT_50HZ" : "IDLE";
}

void printHelp() {
  uPrintln();
  uPrintln("=== DEMO HELP ===");
  uPrint("Mode: "); uPrintln(modeName());
  uPrintln("Keys:");
  uPrintln("  A -> full HW check (SD, BME, HX711, BATT, HALL, GPS)");
  uPrintln("  B -> BME: P, T, RH, ALT");
  uPrintln("  F -> FORCE: HX711 one-shot");
  uPrintln("  H -> HALL one-shot");
  uPrintln("  G -> GPS status");
  uPrintln("  P -> Plot mode (~50Hz): force,hall (CSV) (any other key exits)");
  uPrintln("  L -> LoRa SPI alive test (RegVersion)");
  uPrintln("  T -> LoRa TX test packet (1 packet)");
  uPrintln("  ? / M -> this HELP");
  uPrintln("BT:");
  uPrint("  name: "); uPrintln(BT_NAME);
  uPrint("  connected: "); uPrintln(btConnected() ? "YES" : "NO");
  uPrintln("===============");
  uPrintln();
}

void exitPlotModeIfNeeded(char c) {
  if (mode == MODE_PLOT_50HZ) {
    if (c != 'P' && c != 'p') {
      mode = MODE_IDLE;
      uPrintln();
      uPrintln("Plot mode STOP.");
      printHelp();
    }
  }
}

void runFullCheckA() {
  uPrintln();
  uPrintln("=== FULL HARDWARE CHECK (A) ===");

  // SD
  uPrint("SD: ");
  uPrintln(sdOk ? "OK" : "FAILED");
  if (sdOk) {
    uint8_t cardType = SD.cardType();
    uPrint("SD: type=");
    if (cardType == CARD_NONE) uPrintln("NONE");
    else if (cardType == CARD_MMC) uPrintln("MMC");
    else if (cardType == CARD_SD) uPrintln("SDSC");
    else if (cardType == CARD_SDHC) uPrintln("SDHC/SDXC");
    else uPrintln("UNKNOWN");

    uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
    uPrint("SD: size=");
    uPrint((long)cardSizeMB);
    uPrintln(" MB");

    listDirRootBrief();

    const char* testFile = "/demo_sd_test.txt";
    String line = "SlinkySpace demo write OK, millis=" + String(millis());
    uPrint("SD: R/W test... ");
    bool w = sdWriteTest(testFile, line);
    String r = sdReadAll(testFile);
    uPrintln((w && r.length() > 0) ? "OK" : "FAILED");
  }

  // BME
  uPrint("BME280: ");
  if (bmeOk) {
    uPrintln("OK");
    printBmeOnce();
  } else {
    uPrintln("FAILED (check SDA=32 SCL=33, addr 0x76/0x77, 3.3V)");
  }

  // HX711
  uPrint("HX711: ");
  if (hxOk) {
    uPrintln("OK");
    uPrint("HX711: offset=");
    uPrintln((long)scale.get_offset());
    printForceOnce();
  } else {
    uPrintln("FAILED (check DOUT=22 SCK=27, 3.3V, GND, RATE/jumper)");
  }

  // Battery + Hall
  uPrintln("ADC: OK (configured 12-bit, 11dB)");
  printBatteryOnce();
  printHallOnce();

  // GPS
  uPrint("GPS link: ");
  uPrintln(gpsEverHadChars ? "OK (chars received)" : "NO CHARS YET (check GPS TX->GPIO16, baud 9600)");
  printGpsStatus();

  // LoRa
  uPrint("LoRa init: ");
  uPrintln(loraOk ? "OK (use L/T)" : "FAILED (use L to debug SPI)");

  uPrint("BT connected: ");
  uPrintln(btConnected() ? "YES" : "NO");

  uPrintln("=== CHECK END ===");
  uPrintln();
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=== ESP32 Lolin32 Lite - DEMO (SD + HX711 + BME280 + GPS + BATT + HALL + LoRa) ===");

  // Bluetooth SPP (optional)
  btInitOk = SerialBT.begin(BT_NAME);
  Serial.print("Init BT SPP... ");
  Serial.println(btInitOk ? "OK" : "FAILED");
  Serial.print("BT name: ");
  Serial.println(BT_NAME);

  // I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // GPS UART2
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX2, PIN_GPS_TX2);

  // ADC
  pinMode(PIN_BATT_ADC, INPUT);
  pinMode(PIN_HALL_ADC, INPUT);
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_HALL_ADC, ADC_11db);

  // SPI chip selects idle-high
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  pinMode(PIN_LORA_CS, OUTPUT);
  digitalWrite(PIN_LORA_CS, HIGH);

  // SD init (also starts VSPI pins)
  Serial.print("Init SD... ");
  sdOk = initSD();
  Serial.println(sdOk ? "OK" : "FAILED");

  // BME init
  Serial.print("Init BME280... ");
  bmeOk = initBME();
  if (bmeOk) {
    setupBME();
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // HX init
  Serial.print("Init HX711... ");
  hxOk = initHX711();
  Serial.println(hxOk ? "OK (tared)" : "FAILED");

  // LoRa init (TX)
  Serial.print("Init LoRa TX... ");
  loraOk = initLoRaTX();
  Serial.println(loraOk ? "OK" : "FAILED");

  // Hall baseline
  hallZeroRaw = readHallRaw(40);

  printHelp();
}

void loop() {
  gpsPump(10); // keep GPS parsing alive

  // command from USB or BT
  char c;
  if (readCommandChar(c)) {
    if (c == '\n' || c == '\r') return;

    exitPlotModeIfNeeded(c);

    switch (c) {
      case '?':
      case 'M':
      case 'm':
        printHelp();
        break;

      case 'A':
      case 'a':
        mode = MODE_IDLE;
        runFullCheckA();
        break;

      case 'B':
      case 'b':
        mode = MODE_IDLE;
        printBmeOnce();
        break;

      case 'F':
      case 'f':
        mode = MODE_IDLE;
        printForceOnce();
        break;

      case 'H':
      case 'h':
        mode = MODE_IDLE;
        printHallOnce();
        break;

      case 'G':
      case 'g':
        mode = MODE_IDLE;
        printGpsStatus();
        break;

      case 'L':
      case 'l':
        mode = MODE_IDLE;
        loraSpiAliveTest();
        break;

      case 'T':
      case 't':
        mode = MODE_IDLE;
        loraSendTest();
        break;

      case 'P':
      case 'p':
        if (mode != MODE_PLOT_50HZ) {
          mode = MODE_PLOT_50HZ;
          uPrintln("Plot mode START (~50Hz). Output: force,hall");
          uPrintln("Tip: open Serial Plotter (USB) or BT serial plot app. Press any other key to exit.");
        }
        break;

      default:
        uPrint("Unknown key: ");
        uPrintln(String(c));
        printHelp();
        break;
    }
  }

  // plot stream (~50 Hz)
  if (mode == MODE_PLOT_50HZ) {
    uint32_t now = millis();
    if (now - lastPlotMs >= PLOT_PERIOD_MS) {
      lastPlotMs = now;

      long force = hxReadZeroed();
      if (force == LONG_MIN) force = hxReadRaw();
      if (force == LONG_MIN) force = 0;

      float hall = readHallRaw(8);

      // CSV: force,hall  (USB always, BT only if connected)
      uPrint(force);
      uPrint(',');
      uPrintln((int)hall);
    }
  }
}
