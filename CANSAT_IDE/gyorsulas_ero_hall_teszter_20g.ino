/*
  CanSat Physics Helper — timeoutos, robusztusabb változat
  ESP32 - Arduino IDE
  Saját CANSAT hardverre

  Fő változtatások az eredetihez képest:
    - I2C timeout beállítva
    - HX711 olvasás csak timeoutos wrapperen át
    - kisebb eséllyel blokkoló kimenet (BT előnyben, de Serialra is tükrözhető)
    - nincs dinamikus String a mérési ciklusban
    - bemeneti parancs fix pufferrel
    - hibaszámlálók
    - túlcsúszott minták kezelése

  Parancsok:
    F10..F100   -> mintavételi frekvencia beállítása
    SA          -> csak ay, m/s^2
    SG          -> csak ay, g
    SF          -> csak HX711, N
    SH          -> csak Hall, V
    SO          -> ay_m/s2 ; HX_N ; Hall_V
    SX / STOP   -> mérés leállítása
    SZ          -> HX tare / nullázás
    H0          -> időbélyeg kikapcsolása
    H1          -> időbélyeg bekapcsolása
    STAT        -> állapot / hibaszámlálók
    HELP        -> súgó
*/

#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <HX711.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// -------------------- GPIO --------------------
#define HALL_ADC_PIN  34
#define HX_DOUT       22
#define HX_SCK        27
#define I2C_SDA       32
#define I2C_SCL       33

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

// -------------------- BT --------------------
BluetoothSerial SerialBT;

// -------------------- HX711 --------------------
HX711 scale;

static float hx_offset_counts     = -2067.00146f;
static float hx_scale_N_per_count = +1.0709759e-06f;

// -------------------- Hall ADC --------------------
static const float ADC_REF_V = 3.3f;
static const int   ADC_MAX   = 4095;

// -------------------- Timeoutok --------------------
static const uint32_t I2C_TIMEOUT_MS        = 20;
static const uint32_t HX_READY_TIMEOUT_MS   = 12;
static const uint32_t HX_TARE_TOTAL_MS      = 800;
static const uint32_t INPUT_CMD_GAP_MS      = 120;

// -------------------- MPU állapot --------------------
uint8_t mpuAddr = 0;
bool mpuOk = false;

// -------------------- Mérési mód --------------------
enum MeasureMode {
  MODE_IDLE,
  MODE_AY_MS2,
  MODE_AY_G,
  MODE_HX,
  MODE_HALL,
  MODE_ALL
};

MeasureMode currentMode = MODE_IDLE;

// -------------------- Mintavétel --------------------
int sampleHz = 50;
uint32_t samplePeriodUs = 20000;
uint32_t nextSampleUs = 0;

// -------------------- Kimeneti opciók --------------------
bool includeTime = false;
bool mirrorToSerial = false;   // BT kliens esetén is tükrözze-e Serialra

// -------------------- Fix parancspuffer --------------------
static char cmdBuffer[24];
static size_t cmdLen = 0;
static uint32_t lastCmdCharMs = 0;

// -------------------- Hibaszámlálók --------------------
volatile uint32_t i2cFailCount = 0;
volatile uint32_t mpuReadFailCount = 0;
volatile uint32_t hxNotReadyCount = 0;
volatile uint32_t hxTimeoutCount = 0;
volatile uint32_t sampleSlipCount = 0;
volatile uint32_t cmdOverflowCount = 0;

// ============================================================
// Kimenet
// ============================================================
void sendOutCStr(const char *s) {
  if (SerialBT.hasClient()) {
    SerialBT.println(s);
    if (mirrorToSerial) Serial.println(s);
  } else {
    Serial.println(s);
  }
}

void sendOutFmt(const char *fmt, ...) {
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  sendOutCStr(line);
}

// ============================================================
// Súgó
// ============================================================
void sendHelp() {
  sendOutCStr("");
  sendOutCStr("Parancsok:");
  sendOutCStr("  F10..F100  - mintaveteli frekvencia");
  sendOutCStr("  SA         - ay m/s^2");
  sendOutCStr("  SG         - ay g");
  sendOutCStr("  SF         - HX711 N");
  sendOutCStr("  SH         - Hall V");
  sendOutCStr("  SO         - ay_ms2;hx_N;hall_V");
  sendOutCStr("  SX / STOP  - leallitas");
  sendOutCStr("  SZ         - HX tare / nullazas");
  sendOutCStr("  H0         - ido kikapcsolva");
  sendOutCStr("  H1         - ido bekapcsolva");
  sendOutCStr("  STAT       - hibaszamlalok");
  sendOutCStr("  HELP       - sugo");
  sendOutCStr("");
}

void sendStats() {
  sendOutFmt("STAT:i2c_fail=%lu;mpu_fail=%lu;hx_not_ready=%lu;hx_timeout=%lu;sample_slip=%lu;cmd_overflow=%lu",
             (unsigned long)i2cFailCount,
             (unsigned long)mpuReadFailCount,
             (unsigned long)hxNotReadyCount,
             (unsigned long)hxTimeoutCount,
             (unsigned long)sampleSlipCount,
             (unsigned long)cmdOverflowCount);
}

// ============================================================
// I2C segédfüggvények
// ============================================================
bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  int rc = Wire.endTransmission();
  if (rc != 0) {
    i2cFailCount++;
    return false;
  }
  return true;
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  int rc = Wire.endTransmission(false);
  if (rc != 0) {
    i2cFailCount++;
    return false;
  }

  size_t n = Wire.requestFrom((int)addr, (int)len);
  if (n != len) {
    i2cFailCount++;
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    if (!Wire.available()) {
      i2cFailCount++;
      return false;
    }
    buf[i] = Wire.read();
  }
  return true;
}

// ============================================================
// MPU detektálás és inicializálás
// ============================================================
bool detectAndInitMPU() {
  uint8_t who = 0;

  if (i2cReadBytes(MPU_ADDR0, REG_WHO_AM_I, &who, 1)) {
    if (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73) {
      mpuAddr = MPU_ADDR0;
    }
  }

  if (mpuAddr == 0 && i2cReadBytes(MPU_ADDR1, REG_WHO_AM_I, &who, 1)) {
    if (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73) {
      mpuAddr = MPU_ADDR1;
    }
  }

  if (mpuAddr == 0) return false;

  if (!i2cWriteByte(mpuAddr, REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);
  if (!i2cWriteByte(mpuAddr, REG_PWR_MGMT_2, 0x00)) return false;
  if (!i2cWriteByte(mpuAddr, REG_CONFIG, 0x03)) return false;
  if (!i2cWriteByte(mpuAddr, REG_ACCEL_CONFIG2, 0x03)) return false;
  if (!i2cWriteByte(mpuAddr, REG_GYRO_CONFIG, 0x00)) return false;
  if (!i2cWriteByte(mpuAddr, REG_ACCEL_CONFIG, 0x18)) return false;   // ±16g
  if (!i2cWriteByte(mpuAddr, REG_SMPLRT_DIV, 0x00)) return false;

  return true;
}

bool readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t buf[6];
  if (!mpuOk) return false;
  if (!i2cReadBytes(mpuAddr, REG_ACCEL_XOUT_H, buf, 6)) {
    mpuReadFailCount++;
    return false;
  }

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  return true;
}

// ============================================================
// Szenzorolvasások
// ============================================================
float readAy_g() {
  int16_t ax, ay, az;
  if (!readAccelRaw(ax, ay, az)) return NAN;
  return (float)ay / 2048.0f;   // ±16g -> 2048 LSB/g
}

float readAy_ms2() {
  float ay_g = readAy_g();
  if (isnan(ay_g)) return NAN;
  return ay_g * 10.0f;          // tanórai egyszerűsítés
}

bool hxWaitReady(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (!scale.is_ready()) {
    if ((millis() - t0) >= timeoutMs) {
      hxTimeoutCount++;
      return false;
    }
    delay(1);
  }
  return true;
}

float readHx_N() {
  if (!scale.is_ready()) {
    hxNotReadyCount++;
    if (!hxWaitReady(HX_READY_TIMEOUT_MS)) return NAN;
  }

  long raw = scale.read();
  return ((float)raw - hx_offset_counts) * hx_scale_N_per_count;
}

float readHall_V() {
  int adc = analogRead(HALL_ADC_PIN);
  return ((float)adc * ADC_REF_V) / (float)ADC_MAX;
}

// ============================================================
// Beállítások
// ============================================================
void setFrequency(int hz) {
  sampleHz = hz;
  samplePeriodUs = 1000000UL / (uint32_t)sampleHz;
  sendOutFmt("OK: frekvencia = %d Hz", sampleHz);
}

void startMode(MeasureMode mode, const char *name) {
  currentMode = mode;
  nextSampleUs = micros();
  sendOutFmt("OK: mod = %s", name);
}

void stopMode() {
  currentMode = MODE_IDLE;
  sendOutCStr("OK: meres leallitva");
}

void tareHX() {
  const int TARGET_SAMPLES = 15;
  double sum = 0.0;
  int cnt = 0;
  uint32_t t0 = millis();

  while (cnt < TARGET_SAMPLES && (millis() - t0) < HX_TARE_TOTAL_MS) {
    if (scale.is_ready()) {
      long raw = scale.read();
      sum += (double)raw;
      cnt++;
      delay(20);
    } else {
      delay(2);
    }
  }

  if (cnt > 0) {
    hx_offset_counts = (float)(sum / cnt);
    sendOutFmt("OK: uj HX offset = %.3f (%d minta)", hx_offset_counts, cnt);
  } else {
    hxTimeoutCount++;
    sendOutCStr("HIBA: HX711 timeout / nem olvashato");
  }
}

bool parseFrequencyCommand(const char *cmd, int &hzOut) {
  if (cmd[0] != 'F') return false;
  int hz = atoi(cmd + 1);
  if (hz < 10 || hz > 100) {
    sendOutCStr("HIBA: csak 10..100 Hz lehet");
    return true;
  }
  hzOut = hz;
  return true;
}

// ============================================================
// Parancsfeldolgozás
// ============================================================
void processCommand(const char *cmdRaw) {
  char cmd[24];
  strncpy(cmd, cmdRaw, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // trim + uppercase egyszerűen
  size_t len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\r' || cmd[len - 1] == '\n' || cmd[len - 1] == '\t')) {
    cmd[--len] = '\0';
  }
  size_t start = 0;
  while (cmd[start] == ' ' || cmd[start] == '\t') start++;
  if (start > 0) memmove(cmd, cmd + start, strlen(cmd + start) + 1);
  for (size_t i = 0; cmd[i]; i++) cmd[i] = (char)toupper((unsigned char)cmd[i]);

  if (cmd[0] == '\0') return;

  if (strcmp(cmd, "HELP") == 0) { sendHelp(); return; }
  if (strcmp(cmd, "STAT") == 0) { sendStats(); return; }
  if (strcmp(cmd, "H0") == 0)   { includeTime = false; sendOutCStr("OK: ido kikapcsolva"); return; }
  if (strcmp(cmd, "H1") == 0)   { includeTime = true;  sendOutCStr("OK: ido bekapcsolva"); return; }
  if (strcmp(cmd, "SA") == 0)   { startMode(MODE_AY_MS2, "SA"); return; }
  if (strcmp(cmd, "SG") == 0)   { startMode(MODE_AY_G, "SG"); return; }
  if (strcmp(cmd, "SF") == 0)   { startMode(MODE_HX, "SF"); return; }
  if (strcmp(cmd, "SH") == 0)   { startMode(MODE_HALL, "SH"); return; }
  if (strcmp(cmd, "SO") == 0)   { startMode(MODE_ALL, "SO"); return; }
  if (strcmp(cmd, "SX") == 0 || strcmp(cmd, "STOP") == 0) { stopMode(); return; }
  if (strcmp(cmd, "SZ") == 0)   { tareHX(); return; }

  int hz = 0;
  if (parseFrequencyCommand(cmd, hz)) {
    setFrequency(hz);
    return;
  }

  sendOutFmt("Ismeretlen parancs: %s", cmd);
}

// ============================================================
// Serial / Bluetooth input
// ============================================================
void commitCmdBuffer() {
  if (cmdLen == 0) return;
  cmdBuffer[cmdLen] = '\0';
  processCommand(cmdBuffer);
  cmdLen = 0;
  cmdBuffer[0] = '\0';
}

void handleIncomingChar(char c) {
  lastCmdCharMs = millis();

  if (c == '\n' || c == '\r') {
    commitCmdBuffer();
    return;
  }

  if ((uint8_t)c < 32 || (uint8_t)c > 126) return;

  if (cmdLen < sizeof(cmdBuffer) - 1) {
    cmdBuffer[cmdLen++] = c;
    cmdBuffer[cmdLen] = '\0';
  } else {
    cmdOverflowCount++;
    cmdLen = 0;
    cmdBuffer[0] = '\0';
    sendOutCStr("HIBA: parancsbuffer tulcsordulas");
  }
}

void pollInputs() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    handleIncomingChar(c);
  }

  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    handleIncomingChar(c);
  }

  if (cmdLen > 0 && (millis() - lastCmdCharMs) > INPUT_CMD_GAP_MS) {
    commitCmdBuffer();
  }
}

// ============================================================
// Mérési ciklus
// ============================================================
void doMeasurement() {
  if (currentMode == MODE_IDLE) return;

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) return;

  // ha nagyon lemaradtunk, ne próbáljuk utolérni végtelen sok lépéssel
  if ((uint32_t)(nowUs - nextSampleUs) > (samplePeriodUs * 3UL)) {
    sampleSlipCount++;
    nextSampleUs = nowUs + samplePeriodUs;
  } else {
    nextSampleUs += samplePeriodUs;
  }

  uint32_t tms = millis();
  char line[96];
  line[0] = '\0';

  switch (currentMode) {
    case MODE_AY_MS2: {
      float ay = readAy_ms2();
      if (includeTime) {
        if (isnan(ay)) snprintf(line, sizeof(line), "%lu;NA", (unsigned long)tms);
        else           snprintf(line, sizeof(line), "%lu;%.3f", (unsigned long)tms, ay);
      } else {
        if (isnan(ay)) snprintf(line, sizeof(line), "NA");
        else           snprintf(line, sizeof(line), "%.3f", ay);
      }
      sendOutCStr(line);
      break;
    }

    case MODE_AY_G: {
      float ay = readAy_g();
      if (includeTime) {
        if (isnan(ay)) snprintf(line, sizeof(line), "%lu;NA", (unsigned long)tms);
        else           snprintf(line, sizeof(line), "%lu;%.3f", (unsigned long)tms, ay);
      } else {
        if (isnan(ay)) snprintf(line, sizeof(line), "NA");
        else           snprintf(line, sizeof(line), "%.3f", ay);
      }
      sendOutCStr(line);
      break;
    }

    case MODE_HX: {
      float hx = readHx_N();
      if (includeTime) {
        if (isnan(hx)) snprintf(line, sizeof(line), "%lu;NA", (unsigned long)tms);
        else           snprintf(line, sizeof(line), "%lu;%.5f", (unsigned long)tms, hx);
      } else {
        if (isnan(hx)) snprintf(line, sizeof(line), "NA");
        else           snprintf(line, sizeof(line), "%.5f", hx);
      }
      sendOutCStr(line);
      break;
    }

    case MODE_HALL: {
      float hv = readHall_V();
      if (includeTime) snprintf(line, sizeof(line), "%lu;%.4f", (unsigned long)tms, hv);
      else             snprintf(line, sizeof(line), "%.4f", hv);
      sendOutCStr(line);
      break;
    }

    case MODE_ALL: {
      float ay = readAy_ms2();
      float hx = readHx_N();
      float hv = readHall_V();

      if (includeTime) {
        snprintf(line, sizeof(line), "%lu;%s;%s;%.4f",
                 (unsigned long)tms,
                 isnan(ay) ? "NA" : "",
                 isnan(hx) ? "NA" : "");

        // biztonságosabb, külön építés
        if (isnan(ay) && isnan(hx)) {
          snprintf(line, sizeof(line), "%lu;NA;NA;%.4f", (unsigned long)tms, hv);
        } else if (isnan(ay)) {
          snprintf(line, sizeof(line), "%lu;NA;%.5f;%.4f", (unsigned long)tms, hx, hv);
        } else if (isnan(hx)) {
          snprintf(line, sizeof(line), "%lu;%.3f;NA;%.4f", (unsigned long)tms, ay, hv);
        } else {
          snprintf(line, sizeof(line), "%lu;%.3f;%.5f;%.4f", (unsigned long)tms, ay, hx, hv);
        }
      } else {
        if (isnan(ay) && isnan(hx)) {
          snprintf(line, sizeof(line), "NA;NA;%.4f", hv);
        } else if (isnan(ay)) {
          snprintf(line, sizeof(line), "NA;%.5f;%.4f", hx, hv);
        } else if (isnan(hx)) {
          snprintf(line, sizeof(line), "%.3f;NA;%.4f", ay, hv);
        } else {
          snprintf(line, sizeof(line), "%.3f;%.5f;%.4f", ay, hx, hv);
        }
      }
      sendOutCStr(line);
      break;
    }

    default:
      break;
  }
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1200);

  SerialBT.begin("CanSat_Physics");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  analogReadResolution(12);
  analogSetPinAttenuation(HALL_ADC_PIN, ADC_11db);

  scale.begin(HX_DOUT, HX_SCK);

  mpuOk = detectAndInitMPU();

  sendOutCStr("");
  sendOutCStr("CanSat Physics Helper - timeoutos valtozat");
  sendOutCStr("Varakozas parancsra...");
  sendOutCStr("Alap frekvencia: 50 Hz");
  sendOutCStr("Idobelyeg: OFF");

  if (mpuOk) {
    sendOutFmt("MPU: OK, cim = 0x%02X", mpuAddr);
    sendOutCStr("MPU accel range: +/-16g");
  } else {
    sendOutCStr("MPU: HIBA");
  }

  if (scale.is_ready()) sendOutCStr("HX711: OK");
  else                  sendOutCStr("HX711: nem valaszol");

  sendOutCStr("Hall ADC: OK");
  sendOutFmt("HX offset kezdetben: %.3f", hx_offset_counts);
  sendOutFmt("HX scale N/count: %.10f", hx_scale_N_per_count);
  sendHelp();
}

void loop() {
  pollInputs();
  doMeasurement();
}
