/*
  ESP32 (Wemos Lolin32 Lite) - Demo for CanSat system
  Serial commands:
    A -> one-shot full hardware check (SD, BMP, HX711, Battery, Hall, GPS)  (LoRa not tested)
    B -> BMP: Pressure, Temperature, Altitude
    F -> Force (HX711) one-shot
    H -> Hall one-shot
    G -> GPS status (UTC, location, etc.)
    P -> ~50 Hz stream: force,hall  (CSV for Arduino Plotter)
    ? / M -> HELP menu (anytime)

  Pinout from project:
    SD CS   = GPIO5
    SD SCK  = GPIO18
    SD MISO = GPIO23   (swapped fixed)
    SD MOSI = GPIO19   (swapped fixed)

  Peripherals:
    BMP280 I2C: SDA=GPIO32, SCL=GPIO33
    HX711: DOUT=GPIO22, SCK=GPIO27
    Hall analog: GPIO34  (3.3V)
    Battery divider analog: GPIO35
    LoRa UART: RX=GPIO26, TX=GPIO25  (NOT tested)
    GPS UART2: RX=GPIO16, TX=GPIO17
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <Adafruit_BMP280.h>
#include <HX711.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

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

// UART mapping
static const int PIN_LORA_TX      = 25;
static const int PIN_LORA_RX      = 26;

static const int PIN_GPS_RX2      = 16;
static const int PIN_GPS_TX2      = 17;

// ---------------- Objects ----------------
Adafruit_BMP280 bmp;
bool bmpOk = false;

HX711 scale;
bool hxOk = false;

TinyGPSPlus gps;

HardwareSerial LoRaSerial(1);
HardwareSerial GPSSerial(2);

// Use VSPI explicitly for SD
SPIClass vspi(VSPI);
bool sdOk = false;

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
  if (v < 10) Serial.print('0');
  Serial.print(v);
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
    Serial.println("SD: root open FAILED");
    return;
  }
  Serial.println("SD: Listing / (brief)");
  File entry;
  int count = 0;
  while ((entry = root.openNextFile())) {
    Serial.print("  ");
    Serial.print(entry.name());
    if (entry.isDirectory()) Serial.println("/");
    else {
      Serial.print(" (");
      Serial.print(entry.size());
      Serial.println(" B)");
    }
    entry.close();
    if (++count >= 8) {
      Serial.println("  ...");
      break;
    }
  }
  root.close();
}

bool initSD() {
  vspi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  return SD.begin(PIN_SD_CS, vspi, 20000000);
}

// ---------------- BMP280 ----------------
bool initBMP() {
  if (bmp.begin(0x76)) return true;
  if (bmp.begin(0x77)) return true;
  return false;
}

void setupBMP() {
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_125
  );
  delay(50);
}

void printBmpOnce() {
  if (!bmpOk) {
    Serial.println("BMP: NOT OK (init failed)");
    return;
  }
  float t = bmp.readTemperature();
  float p_hPa = bmp.readPressure() / 100.0f;
  float alt = bmp.readAltitude(SEA_LEVEL_HPA);

  Serial.print("BMP: T=");
  Serial.print(t, 2);
  Serial.print(" C, P=");
  Serial.print(p_hPa, 2);
  Serial.print(" hPa, ALT=");
  Serial.print(alt, 1);
  Serial.println(" m");
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
    Serial.println("HX711: NOT OK (no ready)");
    return;
  }
  long raw = hxReadRaw();
  long zeroed = hxReadZeroed();

  Serial.print("FORCE(HX711): RAW=");
  Serial.print(raw == LONG_MIN ? 0 : raw);

  Serial.print("  ZEROED=");
  Serial.print(zeroed == LONG_MIN ? 0 : zeroed);

  Serial.println();
}

// ---------------- Battery + Hall ----------------
void printBatteryOnce() {
  float adcRaw = readAdcAveraged(PIN_BATT_ADC, 40);
  float v_adc = (adcRaw / ADC_MAX) * ADC_REF_V;

  float divider = (R1_OHMS + R2_OHMS) / R2_OHMS; // 2
  float v_bat = v_adc * divider * VBAT_CAL;
  int pct = estimateLiIonPercent(v_bat);

  Serial.print("BATT: ADCraw=");
  Serial.print(adcRaw, 1);
  Serial.print("  Vadc=");
  Serial.print(v_adc, 3);
  Serial.print(" V  Vbat=");
  Serial.print(v_bat, 3);
  Serial.print(" V  ~");
  Serial.print(pct);
  Serial.println("%");
}

float readHallRaw(int samples = 40) {
  return readAdcAveraged(PIN_HALL_ADC, samples);
}

void printHallOnce() {
  float raw = readHallRaw(40);
  float v = (raw / ADC_MAX) * ADC_REF_V;
  if (isnan(hallZeroRaw)) hallZeroRaw = raw;
  float d = raw - hallZeroRaw;

  Serial.print("HALL: ADCraw=");
  Serial.print(raw, 1);
  Serial.print("  V=");
  Serial.print(v, 3);
  Serial.print(" V  dRAW=");
  Serial.println(d, 1);
}

// ---------------- GPS ----------------
void printGpsStatus() {
  Serial.println("GPS: --- status ---");

  if (!gpsEverHadChars) {
    Serial.println("GPS: no serial chars yet. (check wiring: GPS TX -> GPIO16, baud 9600, GND common)");
    return;
  }

  Serial.print("GPS: chars=");
  Serial.print(gpsCharsTotal);
  Serial.print("  lastCharAgo(ms)=");
  Serial.println(millis() - gpsLastCharMs);

  Serial.print("GPS: checksum ok sentences=");
  Serial.println(gps.passedChecksum());

  Serial.print("GPS: satellites=");
  if (gps.satellites.isValid()) Serial.println(gps.satellites.value());
  else Serial.println("n/a");

  Serial.print("GPS: HDOP=");
  if (gps.hdop.isValid()) Serial.println(gps.hdop.hdop(), 1);
  else Serial.println("n/a");

  if (gps.location.isValid()) {
    Serial.print("GPS: lat=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" lon=");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("GPS: location=n/a (no fix yet)");
  }

  Serial.print("GPS: alt(m)=");
  if (gps.altitude.isValid()) Serial.println(gps.altitude.meters(), 1);
  else Serial.println("n/a");

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print("GPS: UTC ");
    Serial.print(gps.date.year()); Serial.print("-");
    print2digits(gps.date.month()); Serial.print("-");
    print2digits(gps.date.day()); Serial.print(" ");
    print2digits(gps.time.hour()); Serial.print(":");
    print2digits(gps.time.minute()); Serial.print(":");
    print2digits(gps.time.second());
    Serial.println();
  } else {
    Serial.println("GPS: date/time=n/a");
  }
}

// ---------------- HELP / UI ----------------
const char* modeName() {
  return (mode == MODE_PLOT_50HZ) ? "PLOT_50HZ" : "IDLE";
}

void printHelp() {
  Serial.println();
  Serial.println("=== DEMO HELP ===");
  Serial.print("Mode: "); Serial.println(modeName());
  Serial.println("Keys:");
  Serial.println("  A -> full HW check (SD, BMP, HX711, BATT, HALL, GPS)  (LoRa not tested)");
  Serial.println("  B -> BMP: P, T, ALT");
  Serial.println("  F -> FORCE: HX711 one-shot");
  Serial.println("  H -> HALL one-shot");
  Serial.println("  G -> GPS status");
  Serial.println("  P -> Plot mode (~50Hz): force,hall  (CSV)  (any other key exits)");
  Serial.println("  ? / M -> this HELP");
  Serial.println("===============");
  Serial.println();
}

void exitPlotModeIfNeeded(char c) {
  if (mode == MODE_PLOT_50HZ) {
    if (c != 'P' && c != 'p') {
      mode = MODE_IDLE;
      Serial.println();
      Serial.println("Plot mode STOP.");
      printHelp();
    }
  }
}

void runFullCheckA() {
  Serial.println();
  Serial.println("=== FULL HARDWARE CHECK (A) ===");

  // SD
  Serial.print("SD: ");
  Serial.println(sdOk ? "OK" : "FAILED");
  if (sdOk) {
    uint8_t cardType = SD.cardType();
    Serial.print("SD: type=");
    if (cardType == CARD_NONE) Serial.println("NONE");
    else if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC/SDXC");
    else Serial.println("UNKNOWN");

    uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.print("SD: size=");
    Serial.print(cardSizeMB);
    Serial.println(" MB");

    listDirRootBrief();

    const char* testFile = "/demo_sd_test.txt";
    String line = "SlinkySpace demo write OK, millis=" + String(millis());
    Serial.print("SD: R/W test... ");
    bool w = sdWriteTest(testFile, line);
    String r = sdReadAll(testFile);
    Serial.println((w && r.length() > 0) ? "OK" : "FAILED");
  }

  // BMP
  Serial.print("BMP280: ");
  if (bmpOk) {
    Serial.println("OK");
    printBmpOnce();
  } else {
    Serial.println("FAILED (check SDA=32 SCL=33, addr 0x76/0x77, 3.3V)");
  }

  // HX711
  Serial.print("HX711: ");
  if (hxOk) {
    Serial.println("OK");
    Serial.print("HX711: offset=");
    Serial.println(scale.get_offset());
    printForceOnce();
  } else {
    Serial.println("FAILED (check DOUT=22 SCK=27, 3.3V, GND, RATE/jumper)");
  }

  // Battery + Hall
  Serial.println("ADC: OK (configured 12-bit, 11dB)");
  printBatteryOnce();
  printHallOnce();

  // GPS
  Serial.print("GPS link: ");
  Serial.println(gpsEverHadChars ? "OK (chars received)" : "NO CHARS YET (check GPS TX->GPIO16, baud 9600)");
  printGpsStatus();

  Serial.println("LoRa: NOT TESTED (as requested).");
  Serial.println("=== CHECK END ===");
  Serial.println();
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=== ESP32 Lolin32 Lite - DEMO (SD + HX711 + BMP280 + GPS + BATT + HALL) ===");

  // I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // UARTs
  LoRaSerial.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX2, PIN_GPS_TX2);

  // ADC
  pinMode(PIN_BATT_ADC, INPUT);
  pinMode(PIN_HALL_ADC, INPUT);
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_HALL_ADC, ADC_11db);

  // SD init
  Serial.print("Init SD... ");
  sdOk = initSD();
  Serial.println(sdOk ? "OK" : "FAILED");

  // BMP init
  Serial.print("Init BMP280... ");
  bmpOk = initBMP();
  if (bmpOk) {
    setupBMP();
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // HX init
  Serial.print("Init HX711... ");
  hxOk = initHX711();
  Serial.println(hxOk ? "OK (tared)" : "FAILED");

  // Hall baseline
  hallZeroRaw = readHallRaw(40);

  printHelp();
}

void loop() {
  gpsPump(10); // keep GPS parsing alive

  // serial command
  if (Serial.available()) {
    char c = (char)Serial.read();
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
        printBmpOnce();
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

      case 'P':
      case 'p':
        if (mode != MODE_PLOT_50HZ) {
          mode = MODE_PLOT_50HZ;
          Serial.println("Plot mode START (~50Hz). Output: force,hall");
          Serial.println("Tip: open Tools -> Serial Plotter. Press any other key to exit.");
        }
        break;

      default:
        Serial.print("Unknown key: ");
        Serial.println(c);
        printHelp();
        break;
    }
  }

  // plot stream
  if (mode == MODE_PLOT_50HZ) {
    uint32_t now = millis();
    if (now - lastPlotMs >= PLOT_PERIOD_MS) {
      lastPlotMs = now;

      long force = hxReadZeroed();
      if (force == LONG_MIN) force = hxReadRaw();
      if (force == LONG_MIN) force = 0;

      float hall = readHallRaw(8);

      Serial.print(force);
}
