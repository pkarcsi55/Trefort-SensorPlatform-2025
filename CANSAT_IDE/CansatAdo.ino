/*
  SlinkyCanSat TX — FLIGHT CANDIDATE (cleaned)

  Primary mission:
    - 1 Hz LoRa telemetry
    - Compact LoRa payload
    - GPS encoded as scaled integers (x10000)
    - Frequency and SF selectable from Serial at boot
    - Radio config stored in NVS

  Secondary mission:
    - High-rate acquisition to binary SD log
    - Full text telemetry CSV also stored on SD

  Current architecture:
    - LoRa: 1 Hz
    - SD: full logging
    - GPS: opportunistic
    - Secondary sampling: best-effort, actual measured rate depends on total load
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <LoRa.h>
#include <Preferences.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>
#include <HX711.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

// -------------------- FEATURE SWITCHES --------------------
static const bool ENABLE_LORA          = true;
static const bool ENABLE_HX711         = true;
static const bool ENABLE_MPU           = true;
static const bool ENABLE_PARTIAL_FLUSH = true;
static const bool DEFAULT_DIAG_SERIAL  = true;

// -------------------- BOOT MODES --------------------
enum BootMode : uint8_t {
  BOOT_MODE_FLIGHT = 0,
  BOOT_MODE_SD_SERVICE = 1
};

// -------------------- PINOUT --------------------
#define LORA_SCK   18
#define LORA_MISO  23
#define LORA_MOSI  19
#define LORA_CS    25
#define LORA_RST   14
#define LORA_DIO0  26

#define SD_CS      5

#define I2C_SDA    32
#define I2C_SCL    33

#define GPS_RX     17
#define GPS_TX     16
#define GPS_BAUD   9600

#define BAT_ADC_PIN   35
#define NTC_ADC_PIN   13
#define HALL_ADC_PIN  34

#define HX_DOUT 22
#define HX_SCK  27

// -------------------- RADIO --------------------
static const uint32_t DEFAULT_FREQ_HZ = 868750000UL;
static const int      DEFAULT_SF      = 10;
static const long     LORA_BW         = 125E3;
static const int      LORA_CR_DENOM   = 5;
static const int      LORA_TX_PWR     = 17;

// -------------------- TIMING --------------------
static const uint32_t PRE_PROMPT_DELAY_MS      = 2500;
static const uint32_t SERIAL_PROMPT_TIMEOUT_MS = 8000;
static const uint32_t TX_PERIOD_MS              = 1000;
static const uint32_t NTC_UPDATE_PERIOD_MS      = 2000;
static const uint32_t FAST_SAMPLE_PERIOD_US     = 12500;   // nominal scheduler interval
static const uint32_t SD_SPI_HZ                 = 1000000;
static const uint32_t PARTIAL_FLUSH_MS          = 500;
static const uint32_t BOOT_SERIAL_SETTLE_MS      = 2500;
static const uint32_t POST_CONFIG_SETTLE_MS      = 1500;
static const uint32_t POST_RADIO_INIT_SETTLE_MS  = 300;
static const uint32_t BOOT_MENU_ENTER_WINDOW_MS  = 5000;
static const uint32_t CONFIG_MENU_TIMEOUT_MS     = 30000;
static const uint32_t CONFIG_VALUE_TIMEOUT_MS    = 20000;


// -------------------- PACKET / MISSION --------------------
static const char* PACKET_TYPE = "TX1";
static const char* MISSION_ID  = "M1";
static const float SEA_LEVEL_HPA = 1013.25f;

// -------------------- BATTERY --------------------
static const float BAT_R1_OHMS   = 100000.0f;
static const float BAT_R2_OHMS   = 100000.0f;
static const float BAT_DIV_RATIO = (BAT_R1_OHMS + BAT_R2_OHMS) / BAT_R2_OHMS;
static const float BAT_V_EMPTY   = 3.30f;
static const float BAT_V_FULL    = 4.20f;

// -------------------- NTC --------------------
static const float NTC_R_FIXED = 14920.0f;
static const float NTC_R0      = 10000.0f;
static const float NTC_T0_K    = 298.15f;
static const float NTC_BETA    = 3380.0f;
static const float NTC_VREF_MV = 3305.0f;

// -------------------- MPU --------------------
static const uint8_t MPU_ADDR0 = 0x68;
static const uint8_t MPU_ADDR1 = 0x69;
static const uint8_t REG_WHO_AM_I      = 0x75;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_PWR_MGMT_2    = 0x6C;
static const uint8_t REG_SMPLRT_DIV    = 0x19;
static const uint8_t REG_CONFIG        = 0x1A;
static const uint8_t REG_GYRO_CONFIG   = 0x1B;
static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;

// -------------------- SD FILES --------------------
char LOG_FILE_PRIMARY[24]   = "/M1_0001.csv";
char LOG_FILE_SECONDARY[24] = "/M2_0001.bin";

// -------------------- SECONDARY BUFFER --------------------
#define FAST_BUF_SAMPLES 128

struct FastSample {
  uint32_t t_us;
  int32_t  hx;
  int16_t  ax;
  int16_t  ay;
  int16_t  az;
  int16_t  hall;
} __attribute__((packed));

struct Mission2Header {
  char     magic[4];
  uint16_t version;
  uint16_t recordSize;
};

// -------------------- TELEMETRY SNAPSHOT --------------------
struct PrimarySnapshot {
  uint32_t msNow;
  float batV;
  uint8_t batPct;

  bool  bmeOk;
  bool  pValid;
  bool  tInValid;
  bool  rhValid;
  bool  hBmeValid;
  float p;
  float tIn;
  float rh;
  float hBme;

  bool  tOutValid;
  float tOut;

  bool utcValid;
  char utc[32];

  bool  gpsValid;
  double lat;
  double lon;
  double altGps;
};

// -------------------- GLOBALS --------------------
Preferences prefs;
uint32_t loraFreqHz = DEFAULT_FREQ_HZ;
int      loraSf     = DEFAULT_SF;

Adafruit_BME280 bme;
bool bmeOk = false;

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// GPS freshness cache for compact LoRa line
bool   gpsFreshForTx = false;
double gpsFreshLat   = 0.0;
double gpsFreshLon   = 0.0;
double gpsFreshAlt   = 0.0;

HX711 scale;
bool hxOk = false;
int32_t lastHxValue = 0;
uint32_t lastHxReadUs = 0;

uint8_t mpuAddr = MPU_ADDR0;
bool mpuOk = false;
uint8_t mpuWhoami = 0xFF;
int16_t mpu_ax = 0, mpu_ay = 0, mpu_az = 0;

bool sdOk = false;
File secondaryFile;

float ntcLastC = NAN;
bool ntcValid = false;
uint32_t lastNtcUpdateMs = 0;

uint32_t seq = 0;
uint32_t lastFastSampleUs = 0;
uint32_t lastPartialFlushMs = 0;

// fast buffers
FastSample fastBufA[FAST_BUF_SAMPLES];
FastSample fastBufB[FAST_BUF_SAMPLES];
FastSample* fillBuf  = fastBufA;
FastSample* flushBuf = fastBufB;
uint16_t fillCount  = 0;
uint16_t flushCount = 0;
bool flushPending = false;

// stats
uint32_t fastSamplesAcquired = 0;
uint32_t fastSamplesWritten  = 0;
uint32_t fastOverrunCount    = 0;
uint32_t fastLateSkipCount   = 0;
uint32_t lastDiagMs          = 0;
bool diagSerialEnabled       = DEFAULT_DIAG_SERIAL;
BootMode bootMode            = BOOT_MODE_FLIGHT;

// -------------------- Helpers --------------------
static inline bool isValidFreq(uint32_t f) { return (f >= 863000000UL && f <= 870000000UL); }
static inline bool isValidSf(int sf)       { return (sf >= 8 && sf <= 11); }

static inline float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

uint32_t loadFreqFromNVS() {
  prefs.begin("slinky", true);
  uint32_t f = prefs.getUInt("freq_hz", DEFAULT_FREQ_HZ);
  prefs.end();
  if (!isValidFreq(f)) f = DEFAULT_FREQ_HZ;
  return f;
}

int loadSfFromNVS() {
  prefs.begin("slinky", true);
  int sf = prefs.getInt("sf", DEFAULT_SF);
  prefs.end();
  if (!isValidSf(sf)) sf = DEFAULT_SF;
  return sf;
}

bool loadDiagFromNVS() {
  prefs.begin("slinky", true);
  bool en = prefs.getBool("diag_ser", DEFAULT_DIAG_SERIAL);
  prefs.end();
  return en;
}

void saveFreqToNVS(uint32_t f) {
  prefs.begin("slinky", false);
  prefs.putUInt("freq_hz", f);
  prefs.end();
}

void saveSfToNVS(int sf) {
  prefs.begin("slinky", false);
  prefs.putInt("sf", sf);
  prefs.end();
}

void saveDiagToNVS(bool en) {
  prefs.begin("slinky", false);
  prefs.putBool("diag_ser", en);
  prefs.end();
}

String readLineFromSerialWithTimeout(uint32_t timeoutMs) {
  String s;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();

      if (c == '\r') continue;
      if (c == '\n') {
        s.trim();
        return s;
      }
      s += c;
    }
    delay(5);
  }

  s.trim();
  return s;
}


void serialConfigPrompt(uint32_t &freqHz, int &sf, bool &diagEnabled, BootMode &mode) {
  mode = BOOT_MODE_FLIGHT;

  Serial.println();
  Serial.println("========================================");
  Serial.println(" SlinkyCanSat TX - Boot Menu");
  Serial.println("========================================");
  Serial.print("Current frequency: "); Serial.print(freqHz); Serial.println(" Hz");
  Serial.print("Current SF: "); Serial.println(sf);
  Serial.print("Diagnostic serial: "); Serial.println(diagEnabled ? "ON" : "OFF");
  Serial.println();
  Serial.print("Press ENTER within ");
  Serial.print((unsigned long)(BOOT_MENU_ENTER_WINDOW_MS / 1000UL));
  Serial.println(" seconds to enter setup...");
  Serial.println("(or wait for automatic flight start)");
  Serial.print("> ");

  uint32_t startMs = millis();
  bool enterPressed = false;

  while (millis() - startMs < BOOT_MENU_ENTER_WINDOW_MS) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        enterPressed = true;
        break;
      }
      // Any non-empty input still counts as entering setup; discard rest of the line.
      enterPressed = true;
      while (Serial.available()) {
        char d = (char)Serial.read();
        if (d == '\n') break;
      }
      break;
    }
    if (enterPressed) break;
    delay(10);
  }

  if (!enterPressed) {
    Serial.println();
    Serial.println("No input - starting FLIGHT mode");
    mode = BOOT_MODE_FLIGHT;
    return;
  }

  Serial.println();
  Serial.println("Entering configuration menu...");
  delay(300);

  while (true) {
    Serial.println();
    Serial.println("-------- CONFIGURATION MENU --------");
    Serial.println("1 = Set frequency");
    Serial.println("2 = Set Spreading Factor (SF)");
    Serial.println("3 = Toggle diagnostic serial");
    Serial.println("4 = SD service mode");
    Serial.println("5 = Show current settings");
    Serial.println("0 = Save and start flight");
    Serial.println("------------------------------------");
    Serial.print("Choice: ");

    String cmd = readLineFromSerialWithTimeout(CONFIG_MENU_TIMEOUT_MS);
    cmd.trim();

    if (cmd.length() == 0) {
      Serial.println("Menu timeout -> starting flight.");
      mode = BOOT_MODE_FLIGHT;
      return;
    }

    if (cmd == "1") {
      Serial.print("Enter frequency in Hz (current: ");
      Serial.print(freqHz);
      Serial.print("): ");

      String s = readLineFromSerialWithTimeout(CONFIG_VALUE_TIMEOUT_MS);
      s.trim();
      if (s.length() > 0) {
        uint32_t f = (uint32_t)strtoul(s.c_str(), nullptr, 10);
        if (isValidFreq(f)) {
          freqHz = f;
          Serial.print("Frequency set to: ");
          Serial.println(freqHz);
        } else {
          Serial.println("Invalid frequency, keeping previous value.");
        }
      } else {
        Serial.println("Frequency unchanged.");
      }
    }
    else if (cmd == "2") {
      Serial.print("Enter SF 8..11 (current: ");
      Serial.print(sf);
      Serial.print("): ");

      String s = readLineFromSerialWithTimeout(CONFIG_VALUE_TIMEOUT_MS);
      s.trim();
      if (s.length() > 0) {
        int v = s.toInt();
        if (isValidSf(v)) {
          sf = v;
          Serial.print("SF set to: ");
          Serial.println(sf);
        } else {
          Serial.println("Invalid SF, keeping previous value.");
        }
      } else {
        Serial.println("SF unchanged.");
      }
    }
    else if (cmd == "3") {
      diagEnabled = !diagEnabled;
      Serial.print("Diagnostic serial now: ");
      Serial.println(diagEnabled ? "ON" : "OFF");
    }
    else if (cmd == "4") {
      Serial.println("SD service mode selected.");
      mode = BOOT_MODE_SD_SERVICE;
      return;
    }
    else if (cmd == "5") {
      Serial.println();
      Serial.println("Current settings:");
      Serial.print("  Frequency: "); Serial.print(freqHz); Serial.println(" Hz");
      Serial.print("  SF: "); Serial.println(sf);
      Serial.print("  Diagnostic serial: "); Serial.println(diagEnabled ? "ON" : "OFF");
      Serial.print("  Boot mode if continued now: ");
      Serial.println(mode == BOOT_MODE_SD_SERVICE ? "SD service" : "Flight");
    }
    else if (cmd == "0") {
      Serial.println("Saving selection and starting FLIGHT mode.");
      mode = BOOT_MODE_FLIGHT;
      return;
    }
    else {
      Serial.println("Unknown option.");
    }
  }
}

// -------------------- File numbering --------------------
bool parseIndexedFileName(const char* fullName, const char* prefix, const char* ext, uint16_t &idxOut) {
  if (!fullName || !prefix || !ext) return false;

  const char* name = fullName;
  if (name[0] == '/') name++;

  const size_t prefixLen = strlen(prefix);
  const size_t extLen    = strlen(ext);
  const size_t nameLen   = strlen(name);

  if (nameLen != prefixLen + 4 + extLen) return false;
  if (strncmp(name, prefix, prefixLen) != 0) return false;
  if (strcmp(name + prefixLen + 4, ext) != 0) return false;

  uint16_t idx = 0;
  for (size_t i = 0; i < 4; i++) {
    char c = name[prefixLen + i];
    if (c < '0' || c > '9') return false;
    idx = (uint16_t)(idx * 10 + (c - '0'));
  }

  if (idx == 0) return false;
  idxOut = idx;
  return true;
}

uint16_t findNextLogIndexOnSD() {
  uint16_t maxIdx = 0;
  File root = SD.open("/");
  if (!root) return 1;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      uint16_t idx = 0;
      const char* nm = entry.name();
      if (parseIndexedFileName(nm, "M1_", ".csv", idx) ||
          parseIndexedFileName(nm, "M2_", ".bin", idx)) {
        if (idx > maxIdx) maxIdx = idx;
      }
    }
    entry.close();
  }
  root.close();

  if (maxIdx >= 9999) return 9999;
  return (uint16_t)(maxIdx + 1);
}

void prepareIndexedLogFileNames(uint16_t idx) {
  if (idx < 1) idx = 1;
  if (idx > 9999) idx = 9999;
  snprintf(LOG_FILE_PRIMARY,   sizeof(LOG_FILE_PRIMARY),   "/M1_%04u.csv", idx);
  snprintf(LOG_FILE_SECONDARY, sizeof(LOG_FILE_SECONDARY), "/M2_%04u.bin", idx);
}


// -------------------- SD service mode --------------------
String toUpperTrimmed(String s) {
  s.trim();
  for (size_t i = 0; i < s.length(); i++) s[i] = (char)toupper(s[i]);
  return s;
}

void printSdServiceHelp() {
  Serial.println();
  Serial.println("SD SERVICE COMMANDS");
  Serial.println("  HELP              - command list");
  Serial.println("  INFO              - card status");
  Serial.println("  LS                - list files");
  Serial.println("  DUMP <file>       - send file to PC");
  Serial.println("  REBOOT            - restart ESP32");
  Serial.println();
}

void printSdCardInfo() {
  if (!sdOk) {
    Serial.println("INFO SD=FAIL");
    return;
  }

  uint64_t total = SD.totalBytes();
  uint64_t used  = SD.usedBytes();
  Serial.print("INFO SD=OK total=");
  Serial.print((unsigned long)(total / 1024UL));
  Serial.print("KB used=");
  Serial.print((unsigned long)(used / 1024UL));
  Serial.println("KB");
}

void listSdFiles() {
  if (!sdOk) {
    Serial.println("ERR SD not ready");
    return;
  }

  File root = SD.open("/");
  if (!root) {
    Serial.println("ERR open root");
    return;
  }

  Serial.println("LS BEGIN");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      Serial.print(entry.name());
      Serial.print(",");
      Serial.println((unsigned long)entry.size());
    }
    entry.close();
  }
  root.close();
  Serial.println("LS END");
}

void dumpFileToSerial(const String& requestedName) {
  if (!sdOk) {
    Serial.println("ERR SD not ready");
    return;
  }

  String fname = requestedName;
  fname.trim();
  if (fname.length() == 0) {
    Serial.println("ERR missing filename");
    return;
  }
  if (!fname.startsWith("/")) fname = "/" + fname;

  if (secondaryFile) secondaryFile.flush();

  File f = SD.open(fname.c_str(), FILE_READ);
  if (!f) {
    Serial.println("ERR file not found");
    return;
  }

  uint32_t size = (uint32_t)f.size();
  Serial.print("DUMP BEGIN ");
  Serial.print(fname);
  Serial.print(" ");
  Serial.println((unsigned long)size);
  delay(20);

  uint8_t buf[512];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n > 0) Serial.write(buf, n);
  }
  f.close();
  Serial.flush();
  delay(20);
  Serial.println();
  Serial.println("DUMP END");
}

void processSdServiceCommand(const String& line) {
  String cmd = line;
  cmd.trim();
  if (cmd.length() == 0) return;

  String upper = toUpperTrimmed(cmd);

  if (upper == "HELP") {
    printSdServiceHelp();
    return;
  }
  if (upper == "INFO") {
    printSdCardInfo();
    return;
  }
  if (upper == "LS") {
    listSdFiles();
    return;
  }
  if (upper == "REBOOT") {
    Serial.println("REBOOTING");
    delay(200);
    ESP.restart();
    return;
  }
  if (upper.startsWith("DUMP ")) {
    dumpFileToSerial(cmd.substring(5));
    return;
  }

  Serial.println("ERR unknown command");
}

void runSdServiceMode() {
  Serial.println();
  Serial.println("SD service mode active.");
  printSdServiceHelp();

  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      processSdServiceCommand(line);
    }
    delay(5);
  }
}

// -------------------- SPI / SD / LoRa --------------------
void setupSPIShared() {
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
}

void setupSD() {
  Serial.print("microSD... ");

  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  sdOk = SD.begin(SD_CS, SPI, SD_SPI_HZ);
  Serial.println(sdOk ? "OK" : "FAIL");
  if (!sdOk) return;

  uint16_t nextIdx = findNextLogIndexOnSD();
  prepareIndexedLogFileNames(nextIdx);

  File f = SD.open(LOG_FILE_PRIMARY, FILE_WRITE);
  if (!f) {
    Serial.println("ERROR: create primary CSV failed");
    sdOk = false;
    return;
  }
  f.println("PKT,MISSION,SEQ,MS,UTC,P_hPa,T_in_C,RH_pct,T_out_C,H_bme_m,LAT,LON,ALT_gps_m,BAT_V,BAT_PCT");
  f.close();

  f = SD.open(LOG_FILE_SECONDARY, FILE_WRITE);
  if (!f) {
    Serial.println("ERROR: create secondary BIN failed");
    sdOk = false;
    return;
  }

  Mission2Header hdr = {{'M','2','B',0}, 2, sizeof(FastSample)};
  size_t wh = f.write((const uint8_t*)&hdr, sizeof(hdr));
  f.close();
  if (wh != sizeof(hdr)) {
    Serial.println("ERROR: write BIN header failed");
    sdOk = false;
    return;
  }

  secondaryFile = SD.open(LOG_FILE_SECONDARY, FILE_APPEND);
  if (!secondaryFile) {
    Serial.println("ERROR: reopen BIN failed");
    sdOk = false;
    return;
  }

  Serial.print("M1 log: ");
  Serial.println(LOG_FILE_PRIMARY);
  Serial.print("M2 log: ");
  Serial.println(LOG_FILE_SECONDARY);
}

void releaseLoRaFromReset() {
  digitalWrite(LORA_RST, HIGH);
  delay(20);
}

void setupLoRa() {
  if (!ENABLE_LORA) return;

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  Serial.print("LoRa... ");
  if (!LoRa.begin((long)loraFreqHz)) {
    Serial.println("FAIL");
    while (true) delay(1000);
  }
  Serial.println("OK");

  LoRa.enableCrc();
  LoRa.setSpreadingFactor(loraSf);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR_DENOM);
  LoRa.setTxPower(LORA_TX_PWR);

  Serial.print("Radio: ");
  Serial.print(loraFreqHz);
  Serial.print(" Hz, SF");
  Serial.println(loraSf);
}

// -------------------- Sensors --------------------
void setupBME() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  bmeOk = bme.begin(0x76);
  if (!bmeOk) bmeOk = bme.begin(0x77);

  Serial.print("BME280... ");
  Serial.println(bmeOk ? "OK" : "FAIL");
}

void setupGPS() {
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS... OK");
}

void setupBatteryAdc() {
  analogReadResolution(12);
  pinMode(BAT_ADC_PIN, INPUT);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
}

void setupNtcAdc() {
  pinMode(NTC_ADC_PIN, INPUT);
  analogSetPinAttenuation(NTC_ADC_PIN, ADC_11db);
}

void setupHallAdc() {
  pinMode(HALL_ADC_PIN, INPUT);
  analogSetPinAttenuation(HALL_ADC_PIN, ADC_11db);
}

void setupHX711() {
  if (!ENABLE_HX711) {
    hxOk = false;
    return;
  }

  scale.begin(HX_DOUT, HX_SCK);
  if (scale.wait_ready_timeout(1500)) {
    delay(300);
    scale.tare();
    hxOk = true;
    lastHxValue = 0;
    lastHxReadUs = micros();
  } else {
    hxOk = false;
  }

  Serial.print("HX711... ");
  Serial.println(hxOk ? "OK" : "FAIL");
}

// -------------------- MPU --------------------
static bool probeI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)addr, (int)n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  uint8_t b = 0xFF;
  if (!i2cReadN(addr, reg, &b, 1)) return 0xFF;
  return b;
}

bool initMPU() {
  bool has68 = probeI2C(MPU_ADDR0);
  bool has69 = probeI2C(MPU_ADDR1);

  if (has68) mpuAddr = MPU_ADDR0;
  else if (has69) mpuAddr = MPU_ADDR1;
  else return false;

  mpuWhoami = i2cRead8(mpuAddr, REG_WHO_AM_I);
  if (mpuWhoami == 0xFF) return false;

  if (!i2cWrite8(mpuAddr, REG_PWR_MGMT_1, 0x01)) return false;
  delay(10);

  i2cWrite8(mpuAddr, REG_PWR_MGMT_2, 0x00);
  delay(10);

  i2cWrite8(mpuAddr, REG_CONFIG, 0x03);
  i2cWrite8(mpuAddr, REG_SMPLRT_DIV, 9);
  i2cWrite8(mpuAddr, REG_GYRO_CONFIG, 0x18);
  i2cWrite8(mpuAddr, REG_ACCEL_CONFIG, 0x18);
  i2cWrite8(mpuAddr, REG_ACCEL_CONFIG2, 0x03);

  delay(20);
  return true;
}

bool readMPU() {
  uint8_t b[6];

  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((int)mpuAddr, 6);
  if (n != 6) return false;

  for (int i = 0; i < 6; i++) b[i] = Wire.read();

  mpu_ax = (int16_t)((b[0] << 8) | b[1]);
  mpu_ay = (int16_t)((b[2] << 8) | b[3]);
  mpu_az = (int16_t)((b[4] << 8) | b[5]);

  return true;
}

void setupMPU() {
  if (!ENABLE_MPU) {
    mpuOk = false;
    return;
  }

  mpuOk = initMPU();
  Serial.print("MPU... ");
  Serial.println(mpuOk ? "OK" : "FAIL");
}

// -------------------- GPS / ADC --------------------
void pumpGPS_NonBlocking() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  if (gps.location.isUpdated()) {
    gpsFreshLat = gps.location.lat();
    gpsFreshLon = gps.location.lng();
    gpsFreshAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    gpsFreshForTx = true;
  }
}

float readBatteryVoltage() {
  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN);
  float v_adc = (float)mv / 1000.0f;
  return v_adc * BAT_DIV_RATIO;
}

uint8_t batteryPercentFromVoltage(float vbat) {
  float pct = (vbat - BAT_V_EMPTY) / (BAT_V_FULL - BAT_V_EMPTY) * 100.0f;
  pct = clampf(pct, 0.0f, 100.0f);
  return (uint8_t)lroundf(pct);
}

static float ntcResistanceFromVout_mV(float vout_mV) {
  if (vout_mV < 1.0f) vout_mV = 1.0f;
  if (vout_mV > NTC_VREF_MV - 1.0f) vout_mV = NTC_VREF_MV - 1.0f;
  return NTC_R_FIXED * (vout_mV / (NTC_VREF_MV - vout_mV));
}

static float ntcCelsiusFromResistance(float rNtc) {
  float invT = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA) * logf(rNtc / NTC_R0);
  float tK = 1.0f / invT;
  return tK - 273.15f;
}

void updateNtcSlow() {
  uint32_t now = millis();
  if (now - lastNtcUpdateMs < NTC_UPDATE_PERIOD_MS) return;
  lastNtcUpdateMs = now;

  float sum_mV = 0.0f;
  for (int i = 0; i < 4; i++) sum_mV += analogReadMilliVolts(NTC_ADC_PIN);

  float vout_mV = sum_mV / 4.0f;
  float rNtc = ntcResistanceFromVout_mV(vout_mV);
  float tC   = ntcCelsiusFromResistance(rNtc);

  if (isfinite(tC) && tC > -50.0f && tC < 120.0f) {
    ntcLastC = tC;
    ntcValid = true;
  }
}

int16_t readHallFastRaw() {
  int v = analogRead(HALL_ADC_PIN);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

// -------------------- Telemetry snapshot capture --------------------
PrimarySnapshot capturePrimarySnapshot() {
  PrimarySnapshot s;
  memset(&s, 0, sizeof(s));

  s.msNow = millis();

  s.batV = readBatteryVoltage();
  s.batPct = batteryPercentFromVoltage(s.batV);

  s.bmeOk = bmeOk;
  s.pValid = false;
  s.tInValid = false;
  s.rhValid = false;
  s.hBmeValid = false;

  if (bmeOk) {
    float p    = bme.readPressure() / 100.0f;
    float tIn  = bme.readTemperature();
    float rh   = bme.readHumidity();
    float hBme = bme.readAltitude(SEA_LEVEL_HPA);

    if (isfinite(p)) {
      s.p = p;
      s.pValid = true;
    }
    if (isfinite(tIn)) {
      s.tIn = tIn;
      s.tInValid = true;
    }
    if (isfinite(rh)) {
      s.rh = rh;
      s.rhValid = true;
    }
    if (isfinite(hBme)) {
      s.hBme = hBme;
      s.hBmeValid = true;
    }
  }

  s.tOutValid = (ntcValid && isfinite(ntcLastC));
  s.tOut = ntcLastC;

  s.utcValid = false;
  if (gps.time.isValid() && gps.date.isValid()) {
    snprintf(s.utc, sizeof(s.utc), "%04d%02d%02d-%02d%02d%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    s.utcValid = true;
  }

  s.gpsValid = false;
  if (gps.location.isValid()) {
    s.lat = gps.location.lat();
    s.lon = gps.location.lng();
    s.altGps = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    s.gpsValid = true;
  }

  return s;
}

// -------------------- Primary telemetry builders --------------------
String buildFullTelemetryLine(const PrimarySnapshot& s) {
  String batVStr = "NA";
  String batPctStr = "NA";
  if (isfinite(s.batV) && s.batV > 0.5f) {
    batVStr = String(s.batV, 2);
    batPctStr = String((int)s.batPct);
  }

  String pStr    = s.pValid    ? String(s.p, 2)    : "NA";
  String tInStr  = s.tInValid  ? String(s.tIn, 2)  : "NA";
  String rhStr   = s.rhValid   ? String(s.rh, 1)   : "NA";
  String hBmeStr = s.hBmeValid ? String(s.hBme, 1) : "NA";

  String tOutStr = s.tOutValid ? String(s.tOut, 2) : "NA";
  String utcStr  = s.utcValid ? String(s.utc) : "NA";

  String latStr = "NA";
  String lonStr = "NA";
  String altGpsStr = "NA";
  if (s.gpsValid) {
    latStr = String(s.lat, 6);
    lonStr = String(s.lon, 6);
    altGpsStr = String(s.altGps, 1);
  }

  String line;
  line.reserve(260);
  line += PACKET_TYPE; line += ",";
  line += MISSION_ID;  line += ",";
  line += seq;         line += ",";
  line += s.msNow;     line += ",";
  line += utcStr;      line += ",";
  line += pStr;        line += ",";
  line += tInStr;      line += ",";
  line += rhStr;       line += ",";
  line += tOutStr;     line += ",";
  line += hBmeStr;     line += ",";
  line += latStr;      line += ",";
  line += lonStr;      line += ",";
  line += altGpsStr;   line += ",";
  line += batVStr;     line += ",";
  line += batPctStr;

  return line;
}

void buildShortTelemetryLine(char* out, size_t outSize, const PrimarySnapshot& s) {
  char tinBuf[16];
  char toutBuf[16];
  char pBuf[16];
  char ubatBuf[16];

  const char* tinPtr  = "---";
  const char* toutPtr = "---";
  const char* pPtr    = "---";

  if (s.tInValid && s.tIn >= -50.0f && s.tIn <= 100.0f) {
    snprintf(tinBuf, sizeof(tinBuf), "%.1f", s.tIn);
    tinPtr = tinBuf;
  }

  if (s.tOutValid && s.tOut >= -60.0f && s.tOut <= 100.0f) {
    snprintf(toutBuf, sizeof(toutBuf), "%.1f", s.tOut);
    toutPtr = toutBuf;
  }

  if (s.pValid && s.p >= 300.0f && s.p <= 1100.0f) {
    snprintf(pBuf, sizeof(pBuf), "%.1f", s.p);
    pPtr = pBuf;
  }

  if (isfinite(s.batV) && s.batV > 0.5f && s.batV < 6.5f) {
    snprintf(ubatBuf, sizeof(ubatBuf), "%.2f", s.batV);
  } else {
    strcpy(ubatBuf, "---");
  }

  if (gpsFreshForTx) {
    long lat_i = lround(gpsFreshLat * 10000.0);
    long lon_i = lround(gpsFreshLon * 10000.0);
    long alt_i = lround(gpsFreshAlt);

    snprintf(out, outSize,
             "T,%lu,%s,%s,%s,%s,%ld,%ld,%ld",
             (unsigned long)seq,
             tinPtr,
             toutPtr,
             pPtr,
             ubatBuf,
             lat_i,
             lon_i,
             alt_i);
  } else {
    snprintf(out, outSize,
             "T,%lu,%s,%s,%s,%s,---,---,---",
             (unsigned long)seq,
             tinPtr,
             toutPtr,
             pPtr,
             ubatBuf);
  }
}

void logPrimaryToSD_EveryTx(const String& line) {
  if (!sdOk) return;

  digitalWrite(LORA_CS, HIGH);
  File f = SD.open(LOG_FILE_PRIMARY, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

// -------------------- Fast acquisition --------------------
void readHX711IfNeeded() {
  if (!hxOk) return;

  uint32_t nowUs = micros();
  if (nowUs - lastHxReadUs < FAST_SAMPLE_PERIOD_US) return;

  if (scale.is_ready()) {
    long v = scale.get_value(1);
    lastHxValue = (int32_t)(-v);
    lastHxReadUs = nowUs;
  }
}

void queueFilledFastBuffer() {
  if (flushPending) {
    fastOverrunCount++;
    return;
  }

  FastSample* tmp = flushBuf;
  flushBuf = fillBuf;
  fillBuf  = tmp;

  flushCount = fillCount;
  fillCount  = 0;
  flushPending = true;
}

void runFastAcquisition() {
  uint32_t nowUs = micros();
  uint32_t elapsed = nowUs - lastFastSampleUs;

  if (elapsed < FAST_SAMPLE_PERIOD_US) return;

  lastFastSampleUs += FAST_SAMPLE_PERIOD_US;

  if ((nowUs - lastFastSampleUs) > FAST_SAMPLE_PERIOD_US) {
    fastLateSkipCount++;
    lastFastSampleUs = nowUs;
  }

  uint32_t sampleTimeUs = lastFastSampleUs;

  if (fillCount >= FAST_BUF_SAMPLES) {
    queueFilledFastBuffer();
    if (flushPending && fillCount >= FAST_BUF_SAMPLES) {
      fastOverrunCount++;
      return;
    }
  }

  FastSample sample;
  sample.t_us = sampleTimeUs;
  sample.hx   = lastHxValue;
  sample.ax   = 0;
  sample.ay   = 0;
  sample.az   = 0;
  sample.hall = 0;

  if (mpuOk && readMPU()) {
    sample.ax = mpu_ax;
    sample.ay = mpu_ay;
    sample.az = mpu_az;
  }

  sample.hall = readHallFastRaw();

  if (fillCount < FAST_BUF_SAMPLES) {
    fillBuf[fillCount++] = sample;
    fastSamplesAcquired++;
  } else {
    fastOverrunCount++;
    return;
  }

  if (fillCount >= FAST_BUF_SAMPLES) {
    queueFilledFastBuffer();
  }
}

// -------------------- BIN flush --------------------
void flushSecondaryBufferToSD() {
  if (!sdOk || !flushPending || !secondaryFile) return;

  digitalWrite(LORA_CS, HIGH);

  size_t bytesToWrite = sizeof(FastSample) * flushCount;
  size_t written = secondaryFile.write((const uint8_t*)flushBuf, bytesToWrite);
  secondaryFile.flush();

  if (written == bytesToWrite) {
    fastSamplesWritten += flushCount;
  } else if (diagSerialEnabled) {
    Serial.print("WARN flush expected=");
    Serial.print(bytesToWrite);
    Serial.print(" written=");
    Serial.println(written);
  }

  flushCount = 0;
  flushPending = false;
}

void flushSecondaryPartialToSD() {
  if (!ENABLE_PARTIAL_FLUSH) return;
  if (!sdOk || !secondaryFile || flushPending || fillCount == 0) return;

  digitalWrite(LORA_CS, HIGH);

  size_t bytesToWrite = sizeof(FastSample) * fillCount;
  size_t written = secondaryFile.write((const uint8_t*)fillBuf, bytesToWrite);
  secondaryFile.flush();

  if (written == bytesToWrite) {
    fastSamplesWritten += fillCount;
  } else if (diagSerialEnabled) {
    Serial.print("WARN partial expected=");
    Serial.print(bytesToWrite);
    Serial.print(" written=");
    Serial.println(written);
  }

  fillCount = 0;
}

// -------------------- Diagnostics --------------------
void printDiagOncePerSecond() {
  if (!diagSerialEnabled) return;

  static uint32_t prevAcq = 0;
  static uint32_t prevWr  = 0;
  static uint32_t prevOv  = 0;
  static uint32_t prevLate= 0;

  uint32_t nowMs = millis();
  if (nowMs - lastDiagMs < 1000) return;
  lastDiagMs = nowMs;

  uint32_t acqDelta  = fastSamplesAcquired - prevAcq;
  uint32_t wrDelta   = fastSamplesWritten  - prevWr;
  uint32_t ovDelta   = fastOverrunCount    - prevOv;
  uint32_t lateDelta = fastLateSkipCount   - prevLate;

  prevAcq = fastSamplesAcquired;
  prevWr  = fastSamplesWritten;
  prevOv  = fastOverrunCount;
  prevLate= fastLateSkipCount;

  Serial.print("DIAG | acq/s=");
  Serial.print(acqDelta);
  Serial.print(" wr/s=");
  Serial.print(wrDelta);
  Serial.print(" ov/s=");
  Serial.print(ovDelta);
  Serial.print(" late/s=");
  Serial.println(lateDelta);
}

// -------------------- Arduino setup --------------------
void setup() {
  Serial.begin(115200);
  delay(BOOT_SERIAL_SETTLE_MS);

  Serial.println();
  Serial.println("SlinkyCanSat TX boot");
  Serial.println("Opening boot menu...");
  delay(PRE_PROMPT_DELAY_MS);

  loraFreqHz = loadFreqFromNVS();
  loraSf     = loadSfFromNVS();

  diagSerialEnabled = loadDiagFromNVS();

  serialConfigPrompt(loraFreqHz, loraSf, diagSerialEnabled, bootMode);
  saveFreqToNVS(loraFreqHz);
  saveSfToNVS(loraSf);
  saveDiagToNVS(diagSerialEnabled);
  delay(POST_CONFIG_SETTLE_MS);

  setupSPIShared();
  setupSD();

  if (bootMode == BOOT_MODE_SD_SERVICE) {
    runSdServiceMode();
  }

  releaseLoRaFromReset();

  setupBatteryAdc();
  setupNtcAdc();
  setupHallAdc();

  setupBME();
  setupGPS();
  setupHX711();
  setupMPU();
  setupLoRa();

  delay(POST_RADIO_INIT_SETTLE_MS);

  if (sizeof(FastSample) != 16) {
    Serial.println("ERROR: FastSample size != 16");
    while (true) delay(1000);
  }

  lastFastSampleUs = micros();
  lastHxReadUs     = micros();
  lastPartialFlushMs = millis();

  Serial.println("READY");
}

// -------------------- Main loop --------------------
void loop() {
  static uint32_t lastTx = 0;

  // 1) HX711 update
  readHX711IfNeeded();

  // 2) flush completed fast buffer first
  if (flushPending) {
    flushSecondaryBufferToSD();
  }

  // 3) fast sample
  runFastAcquisition();

  // 4) background tasks
  pumpGPS_NonBlocking();
  updateNtcSlow();

  // 5) 1 Hz primary telemetry
  uint32_t nowMs = millis();
  if (nowMs - lastTx >= TX_PERIOD_MS) {
    lastTx = nowMs;

    PrimarySnapshot snap = capturePrimarySnapshot();

    char shortLine[128];
    buildShortTelemetryLine(shortLine, sizeof(shortLine), snap);

    if (ENABLE_LORA) {
      digitalWrite(SD_CS, HIGH);
      LoRa.beginPacket();
      LoRa.print(shortLine);
      LoRa.endPacket();
    }

    Serial.println(shortLine);

    String fullLine = buildFullTelemetryLine(snap);
    logPrimaryToSD_EveryTx(fullLine);

    gpsFreshForTx = false;
    seq++;
  }

  // 6) partial flush
  if (!flushPending && fillCount > 0 && ENABLE_PARTIAL_FLUSH) {
    if (millis() - lastPartialFlushMs > PARTIAL_FLUSH_MS) {
      flushSecondaryPartialToSD();
      lastPartialFlushMs = millis();
    }
  }

  // 7) diagnostics
  printDiagOncePerSecond();
}
