#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>

#include <Wire.h>
#include <Adafruit_BME280.h>

#include <TinyGPSPlus.h>

// -------------------- PINOUT (TX) --------------------
// CANSAT HARDVER
#define LORA_SCK   18
#define LORA_MISO  23
#define LORA_MOSI  19

#define LORA_CS    25
#define LORA_RST   14
#define LORA_DIO0  26

// BME280 I2C
#define I2C_SDA    32
#define I2C_SCL    33

// GPS UART2
#define GPS_RX     16     // ESP32 RX2 <- GPS TX
#define GPS_TX     17     // ESP32 TX2 -> GPS RX (nem mindig kell)
#define GPS_BAUD   9600

// BATTERY ADC (akkufesz osztó)
#define BAT_ADC_PIN 35

// -------------------- RADIO DEFAULT --------------------
static const uint32_t DEFAULT_FREQ_HZ = 868750000UL; // 868.750 MHz

// LoRa paraméterek (egyezzenek a vevővel)
static const int LORA_SF = 9;
static const long LORA_BW = 125E3;
static const int LORA_CR_DENOM = 5; // 4/5
static const int LORA_TX_PWR = 17;  // dBm

// küldési periódus
static const uint32_t TX_PERIOD_MS = 1000;

// -------------------- BATTERY SETTINGS --------------------
// Állítsd a valós ellenállásokra!
// Tipikus példa: R1=100k (akku felől), R2=100k (GND felé) => arány = (R1+R2)/R2 = 2.0
static const float BAT_R1_OHMS = 100000.0f;
static const float BAT_R2_OHMS = 100000.0f;
static const float BAT_DIV_RATIO = (BAT_R1_OHMS + BAT_R2_OHMS) / BAT_R2_OHMS;

// Lineáris %-becslés (Li-ion 1 cella)
static const float BAT_V_EMPTY = 3.30f;
static const float BAT_V_FULL  = 4.20f;

// -------------------- GLOBALS --------------------
Preferences prefs;
uint32_t loraFreqHz = DEFAULT_FREQ_HZ;

Adafruit_BME280 bme;
bool bmeOk = false;

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

uint32_t seq = 0;

// ---------- Helpers ----------
static inline bool isValidFreq(uint32_t f)
{
  // EU868: 863..870 MHz
  return (f >= 863000000UL && f <= 870000000UL);
}

static inline float clampf(float x, float a, float b)
{
  if (x < a) return a;
  if (x > b) return b;
  return x;
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

void maybeChangeFrequencyAtBoot()
{
  loraFreqHz = loadFreqFromNVS();

  Serial.println();
  Serial.println("=== SlinkyCanSat TX boot ===");
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
  LoRa.setTxPower(LORA_TX_PWR);
}

void setupBME()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  bmeOk = bme.begin(0x76);
  if (!bmeOk) bmeOk = bme.begin(0x77);

  Serial.print("BME280 init... ");
  Serial.println(bmeOk ? "OK" : "FAILED (will send NA)");
}

void setupGPS()
{
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART2 init... OK");
}

void setupBatteryAdc()
{
  // ESP32 ADC alap beállítások (stabilabb, ha fixáljuk a felbontást)
  analogReadResolution(12); // 0..4095
  pinMode(BAT_ADC_PIN, INPUT);
  Serial.println("Battery ADC init... OK");
}

void pumpGPS(uint32_t ms)
{
  uint32_t start = millis();
  while (millis() - start < ms) {
    while (GPSSerial.available()) {
      gps.encode(GPSSerial.read());
    }
    delay(1);
  }
}

// ---- Battery read: returns cell voltage in volts ----
float readBatteryVoltage()
{
  // ESP32-n a milliVolts API sokszor kevésbé “szór”
  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN); // mV az ADC lábon (osztó után)
  float v_adc = (float)mv / 1000.0f;
  float v_bat = v_adc * BAT_DIV_RATIO;             // visszaszorozzuk osztóval
  return v_bat;
}

uint8_t batteryPercentFromVoltage(float vbat)
{
  float pct = (vbat - BAT_V_EMPTY) / (BAT_V_FULL - BAT_V_EMPTY) * 100.0f;
  pct = clampf(pct, 0.0f, 100.0f);
  return (uint8_t) lroundf(pct);
}

String buildTelemetryLine()
{
  pumpGPS(5);

  // -------- Battery --------
  float batV = readBatteryVoltage();
  String batVStr = "NA", batPctStr = "NA";
  if (isfinite(batV) && batV > 0.5f) {
    batVStr = String(batV, 2);
    batPctStr = String((int)batteryPercentFromVoltage(batV));
  }

  // -------- BME --------
  String tStr = "NA", pStr = "NA", hStr = "NA";
  if (bmeOk) {
    float t = bme.readTemperature();        // °C
    float p = bme.readPressure() / 100.0f;  // hPa
    float h = bme.readHumidity();           // %
    if (isfinite(t)) tStr = String(t, 2);
    if (isfinite(p)) pStr = String(p, 2);
    if (isfinite(h)) hStr = String(h, 2);
  }

  // -------- GPS --------
  String latStr = "NA", lonStr = "NA", altStr = "NA", utcStr = "NA";

  if (gps.location.isValid() && gps.location.isUpdated()) {
    latStr = String(gps.location.lat(), 6);
    lonStr = String(gps.location.lng(), 6);
  }
  if (gps.altitude.isValid()) {
    altStr = String(gps.altitude.meters(), 1);
  }
  if (gps.time.isValid() && gps.date.isValid()) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d%02d%02d-%02d%02d%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    utcStr = String(buf);
  }

  // -------- Build CSV --------
  // SEQ,MS,UTC,BAT_V,BAT_PCT,T,P,H,LAT,LON,ALT
  String s;
  s.reserve(200);
  s += seq; s += ",";
  s += millis(); s += ",";
  s += utcStr; s += ",";
  s += batVStr; s += ",";
  s += batPctStr; s += ",";
  s += tStr; s += ",";
  s += pStr; s += ",";
  s += hStr; s += ",";
  s += latStr; s += ",";
  s += lonStr; s += ",";
  s += altStr;

  return s;
}

// -------------------- ARDUINO --------------------
void setup()
{
  Serial.begin(115200);
  delay(300);

  maybeChangeFrequencyAtBoot();

  setupBatteryAdc();
  setupBME();
  setupGPS();
  setupLoRa();

  Serial.println("Transmitter ready.");
  Serial.println("Format: SEQ,MS,UTC,BAT_V,BAT_PCT,T,P,H,LAT,LON,ALT");
}

void loop()
{
  static uint32_t lastTx = 0;
  uint32_t now = millis();
  if (now - lastTx < TX_PERIOD_MS) return;
  lastTx = now;

  String line = buildTelemetryLine();

  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();

  Serial.println(line);
  seq++;
}
