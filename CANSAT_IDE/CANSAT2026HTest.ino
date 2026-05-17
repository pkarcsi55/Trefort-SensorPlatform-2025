/*
  SlinkySpace CanSat - ESP32 (Wemos Lolin32 Lite) HARDWARE TESTER (CLEAN EDITION)
  + NTC thermistor test integrated (NTCM-10K-B3380) on GPIO13
  + IMU replaced: MPU-9250 / MPU-6500 / MPU-9255 (I2C, raw registers)

  Notes:
  - NTC divider: 3.3V -> R_FIXED -> Vout(GPIO13) -> NTC -> GND
  - R_FIXED ~ 15k
  - ADC2 on GPIO13: WiFi blocks ADC2. BT SPP is fine.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <Adafruit_BME280.h>
#include <HX711.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#include <LoRa.h>
#include "BluetoothSerial.h"

// ---------------- Bluetooth ----------------
BluetoothSerial SerialBT;
static const char* BT_NAME = "SlinkySpaceDemo";
bool btInitOk = false;

inline bool btConnected() { return btInitOk && SerialBT.hasClient(); }

// text-only helpers (no overload mess)
void uPrint(const char* s) {
  Serial.print(s);
  if (btConnected()) SerialBT.print(s);
}
void uPrintln(const char* s = "") {
  Serial.println(s);
  if (btConnected()) SerialBT.println(s);
}

// numeric helpers (explicit, no ambiguity)
void uPrintNum(long v) { Serial.print(v); if (btConnected()) SerialBT.print(v); }
void uPrintNumUL(unsigned long v) { Serial.print(v); if (btConnected()) SerialBT.print(v); }
void uPrintNumI(int v) { Serial.print(v); if (btConnected()) SerialBT.print(v); }
void uPrintNumF(float v, int digits=3) { Serial.print(v, digits); if (btConnected()) SerialBT.print(v, digits); }

void uPrintlnNum(long v) { Serial.println(v); if (btConnected()) SerialBT.println(v); }
void uPrintlnNumI(int v) { Serial.println(v); if (btConnected()) SerialBT.println(v); }
void uPrintlnNumUL(unsigned long v) { Serial.println(v); if (btConnected()) SerialBT.println(v); }
void uPrintlnNumF(float v, int digits=3) { Serial.println(v, digits); if (btConnected()) SerialBT.println(v, digits); }

// ---------------- Pins ----------------
static const int PIN_HALL_ADC     = 34;
static const int PIN_BATT_ADC     = 35;

// NTC ADC (NEW)
static const int PIN_NTC_ADC      = 13;

static const int PIN_I2C_SDA      = 32;
static const int PIN_I2C_SCL      = 33;

static const int PIN_HX711_DOUT   = 22;
static const int PIN_HX711_SCK    = 27;

static const int PIN_SD_CS        = 5;
static const int PIN_SD_SCK       = 18;
static const int PIN_SD_MISO      = 23;
static const int PIN_SD_MOSI      = 19;

// GPS UART2 (as you measured: GPS RX on GPIO16, TX on GPIO17)
static const int PIN_GPS_RX2      = 17;
static const int PIN_GPS_TX2      = 16;

// LoRa
static const int PIN_LORA_CS      = 25;
static const int PIN_LORA_RST     = 14;
static const int PIN_LORA_DIO0    = 26;

// ---------------- Constants ----------------
static const float SEA_LEVEL_HPA  = 1013.25f;

// ADC
static const int   ADC_BITS       = 12;
static const float ADC_REF_V      = 3.30f;
static const float ADC_MAX        = 4095.0f;

// Battery divider 100k/100k
static const float R1_OHMS        = 100000.0f;
static const float R2_OHMS        = 100000.0f;
static const float VBAT_MIN       = 3.30f;
static const float VBAT_MAX       = 4.20f;
static const float VBAT_CAL       = 1.066f;

// NTC: NTCM-10K-B3380 + 15k fixed resistor
static const float NTC_R_FIXED    = 14920.0f;   // measured ~15k
static const float NTC_R0         = 10000.0f;   // 10k @ 25C
static const float NTC_T0_K       = 298.15f;    // 25C in Kelvin
static const float NTC_BETA       = 3380.0f;    // B3380
static const float NTC_VREF_MV    = 3305.0f;    // measured 3.305V

// GPS
static const uint32_t GPS_BAUD    = 9600;

// streams
static const uint32_t PLOT_50HZ_MS = 20;
static const uint32_t IMU_100HZ_MS = 10;

// ---------------- Objects ----------------
Adafruit_BME280 bme;
bool bmeOk = false;

HX711 scale;
bool hxOk = false;

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

SPIClass vspi(VSPI);
bool sdOk = false;

bool loraOk = false;

// Hall baseline
float hallZeroRaw = NAN;

// GPS bookkeeping
uint32_t gpsCharsTotal = 0;
uint32_t gpsLastCharMs = 0;
bool gpsEverHadChars = false;

// ---------------- MPU-9250/6500/9255 (raw I2C) ----------------
static const uint8_t MPU_ADDR0 = 0x68;
static const uint8_t MPU_ADDR1 = 0x69;

uint8_t mpuAddr = MPU_ADDR0;
bool mpuOk = false;
uint8_t mpuWhoami = 0xFF;

// raw readings
int16_t mpu_ax=0, mpu_ay=0, mpu_az=0;
int16_t mpu_gx=0, mpu_gy=0, mpu_gz=0;
int16_t mpu_temp=0;

// scaling
// for MPU accel full-scale: ±16g -> 2048 LSB/g
static const float MPU_ACC_LSB_PER_G = 2048.0f;
// gyro full-scale: ±2000 dps -> 16.4 LSB/(deg/s)
static const float MPU_GYR_LSB_PER_DPS = 16.4f;

// ---------- I2C helpers ----------
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
  for (uint8_t i=0; i<n; i++) buf[i] = Wire.read();
  return true;
}
static uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  uint8_t b = 0xFF;
  if (!i2cReadN(addr, reg, &b, 1)) return 0xFF;
  return b;
}

// MPU registers
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_PWR_MGMT_2   = 0x6C;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2= 0x1D;
// data block
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

bool initMPU() {
  // detect address 0x68 or 0x69
  bool has68 = probeI2C(MPU_ADDR0);
  bool has69 = probeI2C(MPU_ADDR1);
  if      (has68) mpuAddr = MPU_ADDR0;
  else if (has69) mpuAddr = MPU_ADDR1;
  else return false;

  // read WHO_AM_I
  mpuWhoami = i2cRead8(mpuAddr, REG_WHO_AM_I);
  if (mpuWhoami == 0xFF) return false;

  // wake up: clear sleep, set clock source (PLL gyro X)
  // PWR_MGMT_1: 0x01 -> clock=PLL X gyro, sleep=0
  if (!i2cWrite8(mpuAddr, REG_PWR_MGMT_1, 0x01)) return false;
  delay(10);

  // enable all axes (PWR_MGMT_2 = 0)
  i2cWrite8(mpuAddr, REG_PWR_MGMT_2, 0x00);
  delay(10);

  // DLPF config (CONFIG): ~41 Hz gyro DLPF (value 3) – stabil teszthez jó
  i2cWrite8(mpuAddr, REG_CONFIG, 0x03);

  // sample rate divider: Gyro output rate 1kHz when DLPF on
  // Rate = 1000 / (1 + div) -> 100Hz => div=9
  i2cWrite8(mpuAddr, REG_SMPLRT_DIV, 9);

  // Gyro full-scale ±2000 dps: bits [4:3] = 3 -> 0x18
  i2cWrite8(mpuAddr, REG_GYRO_CONFIG, 0x18);

  // Accel full-scale ±16g: bits [4:3] = 3 -> 0x18
  i2cWrite8(mpuAddr, REG_ACCEL_CONFIG, 0x18);

  // Accel DLPF (ACCEL_CONFIG2): 41 Hz -> 0x03
  i2cWrite8(mpuAddr, REG_ACCEL_CONFIG2, 0x03);

  delay(20);
  return true;
}

bool readMPU() {
  uint8_t b[14];
  if (!i2cReadN(mpuAddr, REG_ACCEL_XOUT_H, b, 14)) return false;

  mpu_ax = (int16_t)((b[0] << 8) | b[1]);
  mpu_ay = (int16_t)((b[2] << 8) | b[3]);
  mpu_az = (int16_t)((b[4] << 8) | b[5]);
  mpu_temp = (int16_t)((b[6] << 8) | b[7]);
  mpu_gx = (int16_t)((b[8] << 8) | b[9]);
  mpu_gy = (int16_t)((b[10] << 8) | b[11]);
  mpu_gz = (int16_t)((b[12] << 8) | b[13]);

  return true;
}

const char* whoamiName(uint8_t who) {
  if (who == 0x70) return "MPU-6500 (WHO_AM_I=0x70)";
  if (who == 0x71) return "MPU-9250 (WHO_AM_I=0x71)";
  if (who == 0x73) return "MPU-9255 (WHO_AM_I=0x73)";
  return "MPU? (unknown WHO_AM_I)";
}

void printMpuOnce() {
  uPrintln("MPU-9250/6500/9255: --- one-shot ---");
  if (!mpuOk) { uPrintln("MPU: NOT OK"); return; }
  if (!readMPU()) { uPrintln("MPU: read FAILED"); return; }

  float ax_g = (float)mpu_ax / MPU_ACC_LSB_PER_G;
  float ay_g = (float)mpu_ay / MPU_ACC_LSB_PER_G;
  float az_g = (float)mpu_az / MPU_ACC_LSB_PER_G;
  float amag_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

  float gx_dps = (float)mpu_gx / MPU_GYR_LSB_PER_DPS;
  float gy_dps = (float)mpu_gy / MPU_GYR_LSB_PER_DPS;
  float gz_dps = (float)mpu_gz / MPU_GYR_LSB_PER_DPS;

  // temp formula (MPU-6050/6500/9250 family): T(°C) = (TEMP_OUT/333.87) + 21
  float tC = ((float)mpu_temp / 333.87f) + 21.0f;

  uPrint("MPU addr: 0x");
  Serial.print(mpuAddr, HEX); if (btConnected()) SerialBT.print(mpuAddr, HEX);
  uPrint("  "); uPrintln(whoamiName(mpuWhoami));

  uPrint("ACC RAW: x="); uPrintNum(mpu_ax);
  uPrint(" y="); uPrintNum(mpu_ay);
  uPrint(" z="); uPrintlnNum(mpu_az);

  uPrint("ACC g  : ax="); uPrintNumF(ax_g,4);
  uPrint(" ay=");         uPrintNumF(ay_g,4);
  uPrint(" az=");         uPrintNumF(az_g,4);
  uPrint(" |a|=");        uPrintNumF(amag_g,4);
  uPrintln(" g");

  uPrint("GYR dps: gx="); uPrintNumF(gx_dps,2);
  uPrint(" gy=");         uPrintNumF(gy_dps,2);
  uPrint(" gz=");         uPrintlnNumF(gz_dps,2);

  uPrint("TEMP: "); uPrintNumF(tC,2); uPrintln(" C");
}

// ---------------- Helpers ----------------
float readAdcMilliVoltsAveraged(int pin, int samples){
  uint32_t sum=0;
  for(int i=0;i<samples;i++){ sum += (uint32_t)analogReadMilliVolts(pin); delay(2); }
  return (float)sum/(float)samples;
}

float clampf(float x,float a,float b){ if(x<a) return a; if(x>b) return b; return x; }

int estimateLiIonPercent(float vbat) {
  float p=(vbat-VBAT_MIN)/(VBAT_MAX-VBAT_MIN)*100.0f;
  p=clampf(p,0.0f,100.0f);
  return (int)(p+0.5f);
}

float readAdcAveraged(int pin,int samples){
  uint32_t sum=0;
  for(int i=0;i<samples;i++){ sum += analogRead(pin); delay(2); }
  return (float)sum/(float)samples;
}

void print2digits(int v){ if(v<10) uPrint("0"); uPrintNumI(v); }

void gpsPump(unsigned long maxMs=10){
  unsigned long start=millis();
  while(millis()-start<maxMs){
    while(GPSSerial.available()){
      char c=(char)GPSSerial.read();
      gps.encode(c);
      gpsCharsTotal++;
      gpsLastCharMs=millis();
      gpsEverHadChars=true;
    }
  }
}

bool readCommandChar(char &c) {
  if (Serial.available()) { c=(char)Serial.read(); return true; }
  if (btConnected() && SerialBT.available()) { c=(char)SerialBT.read(); return true; }
  return false;
}

// ---------------- NTC ----------------
float ntcResistanceFromVout_mV(float vout_mV){
  if(vout_mV < 1.0f) vout_mV = 1.0f;
  if(vout_mV > NTC_VREF_MV - 1.0f) vout_mV = NTC_VREF_MV - 1.0f;
  return NTC_R_FIXED * (vout_mV / (NTC_VREF_MV - vout_mV));
}
float ntcCelsiusFromResistance(float rNtc){
  float invT = (1.0f/NTC_T0_K) + (1.0f/NTC_BETA) * logf(rNtc/NTC_R0);
  float tK = 1.0f / invT;
  return tK - 273.15f;
}
void printNtcOnce(){
  float vout_mV = readAdcMilliVoltsAveraged(PIN_NTC_ADC, 80);
  float rNtc = ntcResistanceFromVout_mV(vout_mV);
  float tC = ntcCelsiusFromResistance(rNtc);

  uPrint("NTC(GPIO13): Vout="); uPrintNumF(vout_mV/1000.0f,3);
  uPrint(" V, R=");            uPrintNumF(rNtc,0);
  uPrint(" ohm, T=");          uPrintNumF(tC,2);
  uPrintln(" C");
}

// ---------------- SD ----------------
bool initSD(){
  vspi.begin(PIN_SD_SCK,PIN_SD_MISO,PIN_SD_MOSI,PIN_SD_CS);
  return SD.begin(PIN_SD_CS,vspi,20000000);
}

void listDirRootBrief(){
  File root=SD.open("/");
  if(!root || !root.isDirectory()){ uPrintln("SD: root open FAILED"); return; }
  uPrintln("SD: Listing / (brief)");
  File entry; int count=0;
  while((entry=root.openNextFile())){
    uPrint("  "); uPrint(entry.name());
    if(entry.isDirectory()) uPrintln("/");
    else { uPrint(" ("); uPrintNum((long)entry.size()); uPrintln(" B)"); }
    entry.close();
    if(++count>=8){ uPrintln("  ..."); break; }
  }
  root.close();
}

bool sdWriteTest(const char* path,const char* text){
  File f=SD.open(path,FILE_WRITE);
  if(!f) return false;
  size_t n=f.println(text);
  f.close();
  return (n>0);
}

String sdReadAll(const char* path){
  File f=SD.open(path,FILE_READ);
  if(!f) return String("");
  String content;
  while(f.available()) content += char(f.read());
  f.close();
  return content;
}

// ---------------- BME ----------------
bool initBME(){
  if(bme.begin(0x76,&Wire)) return true;
  if(bme.begin(0x77,&Wire)) return true;
  return false;
}

void setupBME(){
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X2,
    Adafruit_BME280::SAMPLING_X16,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::FILTER_X16,
    Adafruit_BME280::STANDBY_MS_125
  );
  delay(50);
}

void printBmeOnce(){
  if(!bmeOk){ uPrintln("BME: NOT OK"); return; }
  float t=bme.readTemperature();
  float p_hPa=bme.readPressure()/100.0f;
  float rh=bme.readHumidity();
  float alt=bme.readAltitude(SEA_LEVEL_HPA);

  uPrint("BME: T="); uPrintNumF(t,2);
  uPrint(" C, P=");  uPrintNumF(p_hPa,2);
  uPrint(" hPa, RH="); uPrintNumF(rh,1);
  uPrint(" %, ALT="); uPrintNumF(alt,1);
  uPrintln(" m");
}

// ---------------- HX711 ----------------
bool initHX711(){
  scale.begin(PIN_HX711_DOUT,PIN_HX711_SCK);
  if(!scale.wait_ready_timeout(1500)) return false;
  delay(800);
  scale.tare();
  return true;
}
long hxReadRaw(){
  if(!hxOk) return LONG_MIN;
  if(!scale.wait_ready_timeout(100)) return LONG_MIN;
  return scale.read();
}
long hxReadZeroed(){
  if(!hxOk) return LONG_MIN;
  if(!scale.wait_ready_timeout(100)) return LONG_MIN;
  return scale.get_value(1);
}
void printForceOnce(){
  if(!hxOk){ uPrintln("HX711: NOT OK"); return; }
  long raw=hxReadRaw();
  long zeroed=hxReadZeroed();
  uPrint("FORCE: RAW="); uPrintNum(raw==LONG_MIN?0:raw);
  uPrint(" ZEROED=");    uPrintlnNum(zeroed==LONG_MIN?0:zeroed);
}

// ---------------- Battery + Hall ----------------
void printBatteryOnce(){
  float adcRaw=readAdcAveraged(PIN_BATT_ADC,40);
  float v_adc=(adcRaw/ADC_MAX)*ADC_REF_V;

  float divider=(R1_OHMS+R2_OHMS)/R2_OHMS;
  float v_bat=v_adc*divider*VBAT_CAL;
  int pct=estimateLiIonPercent(v_bat);

  uPrint("BATT: ADC="); uPrintNumF(adcRaw,1);
  uPrint(" Vadc=");     uPrintNumF(v_adc,3);
  uPrint(" Vbat=");     uPrintNumF(v_bat,3);
  uPrint(" ~");         uPrintNumI(pct);
  uPrintln("%");
}

float readHallRaw(int samples=40){ return readAdcAveraged(PIN_HALL_ADC,samples); }

void printHallOnce(){
  float raw=readHallRaw(40);
  float v=(raw/ADC_MAX)*ADC_REF_V;
  if(isnan(hallZeroRaw)) hallZeroRaw=raw;
  float d=raw-hallZeroRaw;

  uPrint("HALL: ADC="); uPrintNumF(raw,1);
  uPrint(" V=");        uPrintNumF(v,3);
  uPrint(" dRAW=");     uPrintlnNumF(d,1);
}

// ---------------- GPS ----------------
void printGpsStatus(){
  uPrintln("GPS: --- status ---");
  if(!gpsEverHadChars){
    uPrintln("GPS: no serial chars yet (check TX->GPIO17, 9600 baud)");
    return;
  }

  uPrint("GPS: chars="); uPrintNumUL(gpsCharsTotal);
  uPrint(" lastCharAgo(ms)="); uPrintlnNumUL(millis()-gpsLastCharMs);

  uPrint("GPS: checksum ok sentences="); uPrintlnNumUL(gps.passedChecksum());

  uPrint("GPS: satellites=");
  if(gps.satellites.isValid()) uPrintlnNumI((int)gps.satellites.value());
  else uPrintln("n/a");

  uPrint("GPS: HDOP=");
  if(gps.hdop.isValid()) { uPrintlnNumF(gps.hdop.hdop(),1); }
  else uPrintln("n/a");

  if(gps.location.isValid()){
    uPrint("GPS: lat="); Serial.print(gps.location.lat(),6); if(btConnected()) SerialBT.print(gps.location.lat(),6);
    uPrint(" lon=");     Serial.println(gps.location.lng(),6); if(btConnected()) SerialBT.println(gps.location.lng(),6);
  } else {
    uPrintln("GPS: location=n/a (no fix yet)");
  }

  uPrint("GPS: alt(m)=");
  if(gps.altitude.isValid()) uPrintlnNumF(gps.altitude.meters(),1);
  else uPrintln("n/a");

  if(gps.date.isValid() && gps.time.isValid()){
    uPrint("GPS: UTC ");
    uPrintNumI((int)gps.date.year()); uPrint("-");
    print2digits((int)gps.date.month()); uPrint("-");
    print2digits((int)gps.date.day()); uPrint(" ");
    print2digits((int)gps.time.hour()); uPrint(":");
    print2digits((int)gps.time.minute()); uPrint(":");
    print2digits((int)gps.time.second());
    uPrintln();
  } else {
    uPrintln("GPS: date/time=n/a");
  }
}

// ---------------- LoRa ----------------
#define REG_VERSION 0x42

uint8_t sx127xReadReg(uint8_t addr){
  digitalWrite(PIN_LORA_CS,LOW);
  vspi.transfer(addr & 0x7F);
  uint8_t v=vspi.transfer(0x00);
  digitalWrite(PIN_LORA_CS,HIGH);
  return v;
}
bool initLoRaTX(){
  LoRa.setSPI(vspi);
  LoRa.setPins(PIN_LORA_CS,PIN_LORA_RST,PIN_LORA_DIO0);
  if(!LoRa.begin(868E6)) return false;

  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  return true;
}
void loraSpiAliveTest(){
  uPrintln("LoRa SPI alive test:");
  uint8_t ver=sx127xReadReg(REG_VERSION);
  uPrint("  RegVersion(0x42)=0x");
  Serial.print(ver,HEX); if(btConnected()) SerialBT.print(ver,HEX);
  uPrintln(ver==0x12 ? " OK(SX1276/78)" : " (check SPI if 00/FF)");
}
void loraSendTest(){
  if(!loraOk){ uPrintln("LoRa TX: NOT OK"); return; }
  uPrint("LoRa TX test... millis="); uPrintlnNumUL(millis());
  LoRa.beginPacket();
  LoRa.print("CANSAT_TX_TEST,");
  LoRa.print(millis());
  int ok = LoRa.endPacket(true);
  uPrint("endPacket="); uPrintlnNumI(ok);
}

// ---------------- HELP ----------------
enum Mode { MODE_IDLE=0, MODE_PLOT_50HZ, MODE_IMU_100HZ };
Mode mode = MODE_IDLE;
uint32_t lastTickMs = 0;

const char* modeName(){
  switch(mode){
    case MODE_PLOT_50HZ: return "PLOT_50HZ";
    case MODE_IMU_100HZ: return "IMU_100HZ";
    default: return "IDLE";
  }
}

void printHelp(){
  uPrintln();
  uPrintln("=== SlinkySpace HW TESTER (CLEAN) ===");
  uPrint("Mode: "); uPrintln(modeName());
  uPrintln("Keys:");
  uPrintln("  A -> full check");
  uPrintln("  B -> BME one-shot");
  uPrintln("  I -> MPU one-shot (acc+gyro)");
  uPrintln("  F -> HX711 one-shot");
  uPrintln("  H -> Hall one-shot");
  uPrintln("  V -> Battery one-shot");
  uPrintln("  N -> NTC one-shot (GPIO13, B3380, 15k)");
  uPrintln("  G -> GPS status");
  uPrintln("  P -> stream 50Hz: force,ax_g,ay_g,az_g,hall");
  uPrintln("  Q -> stream 100Hz: ax_g,ay_g,az_g,amag_g");
  uPrintln("  L -> LoRa SPI alive test");
  uPrintln("  T -> LoRa TX test packet");
  uPrintln("  ?/M -> help");
  uPrint("BT name: "); uPrintln(BT_NAME);
  uPrint("BT connected: "); uPrintln(btConnected() ? "YES" : "NO");
  uPrintln("====================================");
  uPrintln();
}

void exitStreamIfNeeded(char c){
  if(mode==MODE_PLOT_50HZ){
    if(c!='P' && c!='p'){ mode=MODE_IDLE; uPrintln("\nStream STOP (P)."); printHelp(); }
  }
  if(mode==MODE_IMU_100HZ){
    if(c!='Q' && c!='q'){ mode=MODE_IDLE; uPrintln("\nStream STOP (Q)."); printHelp(); }
  }
}

// ---------------- Full check ----------------
void runFullCheckA(){
  uPrintln();
  uPrintln("=== FULL HARDWARE CHECK (A) ===");

  uPrint("SD: "); uPrintln(sdOk ? "OK" : "FAILED");
  if(sdOk){
    uPrint("SD size MB: "); uPrintlnNum((long)(SD.cardSize()/(1024ULL*1024ULL)));
    listDirRootBrief();
    const char* testFile="/demo_sd_test.txt";
    sdWriteTest(testFile, "SlinkySpace demo write OK");
    String r = sdReadAll(testFile);
    uPrint("SD R/W: "); uPrintln(r.length() ? "OK" : "FAILED");
  }

  uPrint("BME280: "); uPrintln(bmeOk ? "OK" : "FAILED");
  if(bmeOk) printBmeOnce();

  uPrint("MPU: "); uPrintln(mpuOk ? "OK" : "FAILED");
  if(mpuOk) printMpuOnce();

  uPrint("HX711: "); uPrintln(hxOk ? "OK" : "FAILED");
  if(hxOk) printForceOnce();

  uPrintln("ADC:");
  printBatteryOnce();
  printHallOnce();
  printNtcOnce();

  uPrint("GPS link: "); uPrintln(gpsEverHadChars ? "OK" : "NO CHARS YET");
  printGpsStatus();

  uPrint("LoRa init: "); uPrintln(loraOk ? "OK" : "FAILED");
  uPrint("BT connected: "); uPrintln(btConnected() ? "YES" : "NO");

  uPrintln("=== CHECK END ===");
  uPrintln();
}

// ---------------- Setup / loop ----------------
void setup(){
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=== SlinkySpace HW TESTER (CLEAN) ===");

  btInitOk = SerialBT.begin(BT_NAME);
  Serial.print("Init BT SPP... "); Serial.println(btInitOk ? "OK" : "FAILED");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX2, PIN_GPS_TX2);

  pinMode(PIN_BATT_ADC, INPUT);
  pinMode(PIN_HALL_ADC, INPUT);
  pinMode(PIN_NTC_ADC,  INPUT);

  analogReadResolution(ADC_BITS);

  // ADC attenuation
  analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_HALL_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_NTC_ADC,  ADC_11db);

  pinMode(PIN_SD_CS, OUTPUT);   digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);

  Serial.print("Init SD... ");
  sdOk = initSD();
  Serial.println(sdOk ? "OK" : "FAILED");

  Serial.print("Init BME280... ");
  bmeOk = initBME();
  if (bmeOk) { setupBME(); Serial.println("OK"); }
  else Serial.println("FAILED");

  Serial.print("Init MPU (9250/6500/9255)... ");
  mpuOk = initMPU();
  if (mpuOk) {
    Serial.print("OK, addr=0x"); Serial.print(mpuAddr, HEX);
    Serial.print(" WHO_AM_I=0x"); Serial.println(mpuWhoami, HEX);
  } else {
    Serial.println("FAILED");
  }

  Serial.print("Init HX711... ");
  hxOk = initHX711();
  Serial.println(hxOk ? "OK (tared)" : "FAILED");

  Serial.print("Init LoRa TX... ");
  loraOk = initLoRaTX();
  Serial.println(loraOk ? "OK" : "FAILED");

  hallZeroRaw = readHallRaw(40);

  printHelp();
}

void loop(){
  gpsPump(10);

  char c;
  if(readCommandChar(c)){
    if(c=='\n' || c=='\r') return;

    exitStreamIfNeeded(c);

    switch(c){
      case '?': case 'M': case 'm': printHelp(); break;
      case 'A': case 'a': mode=MODE_IDLE; runFullCheckA(); break;
      case 'B': case 'b': mode=MODE_IDLE; printBmeOnce(); break;
      case 'I': case 'i': mode=MODE_IDLE; printMpuOnce(); break;
      case 'F': case 'f': mode=MODE_IDLE; printForceOnce(); break;
      case 'H': case 'h': mode=MODE_IDLE; printHallOnce(); break;
      case 'V': case 'v': mode=MODE_IDLE; printBatteryOnce(); break;
      case 'N': case 'n': mode=MODE_IDLE; printNtcOnce(); break;
      case 'G': case 'g': mode=MODE_IDLE; printGpsStatus(); break;
      case 'L': case 'l': mode=MODE_IDLE; loraSpiAliveTest(); break;
      case 'T': case 't': mode=MODE_IDLE; loraSendTest(); break;

      case 'P': case 'p':
        if(mode != MODE_PLOT_50HZ){
          mode = MODE_PLOT_50HZ;
          lastTickMs = millis();
          uPrintln("Stream START 50Hz (P): force,ax_g,ay_g,az_g,hall");
          uPrintln("Press any other key to exit.");
        }
        break;

      case 'Q': case 'q':
        if(mode != MODE_IMU_100HZ){
          if(!mpuOk){ uPrintln("IMU stream: MPU NOT OK"); break; }
          mode = MODE_IMU_100HZ;
          lastTickMs = millis();
          uPrintln("Stream START 100Hz (Q): ax_g,ay_g,az_g,amag_g");
          uPrintln("Press any other key to exit.");
        }
        break;

      default:
        uPrint("Unknown key: "); Serial.println(c); if(btConnected()) SerialBT.println(c);
        printHelp();
        break;
    }
  }

  uint32_t now = millis();

  // P: 50 Hz stream: force,ax,ay,az,hall
  if(mode == MODE_PLOT_50HZ){
    if(now - lastTickMs >= PLOT_50HZ_MS){
      lastTickMs = now;

      // 1) HX711
      long force = hxReadZeroed();
      if(force == LONG_MIN) force = hxReadRaw();
      if(force == LONG_MIN) force = 0;

      // 2) MPU accel (g)
      float ax_g = NAN, ay_g = NAN, az_g = NAN;
      if(mpuOk && readMPU()){
        ax_g = (float)mpu_ax / MPU_ACC_LSB_PER_G;
        ay_g = (float)mpu_ay / MPU_ACC_LSB_PER_G;
        az_g = (float)mpu_az / MPU_ACC_LSB_PER_G;
      }

      // 3) Hall
      float hall = readHallRaw(8);

      // CSV: force,ax_g,ay_g,az_g,hall
      Serial.print(force); Serial.print(',');
      Serial.print(ax_g,4); Serial.print(',');
      Serial.print(ay_g,4); Serial.print(',');
      Serial.print(az_g,4); Serial.print(',');
      Serial.println((int)hall);

      if(btConnected()){
        SerialBT.print(force); SerialBT.print(',');
        SerialBT.print(ax_g,4); SerialBT.print(',');
        SerialBT.print(ay_g,4); SerialBT.print(',');
        SerialBT.print(az_g,4); SerialBT.print(',');
        SerialBT.println((int)hall);
      }
    }
  }

  // Q: 100 Hz stream: ax,ay,az,|a|
  if(mode == MODE_IMU_100HZ){
    if(now - lastTickMs >= IMU_100HZ_MS){
      lastTickMs = now;

      if(!readMPU()) return;

      float ax_g = (float)mpu_ax / MPU_ACC_LSB_PER_G;
      float ay_g = (float)mpu_ay / MPU_ACC_LSB_PER_G;
      float az_g = (float)mpu_az / MPU_ACC_LSB_PER_G;
      float amag_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

      Serial.print(ax_g,4); Serial.print(',');
      Serial.print(ay_g,4); Serial.print(',');
      Serial.print(az_g,4); Serial.print(',');
      Serial.println(amag_g,4);

      if(btConnected()){
        SerialBT.print(ax_g,4); SerialBT.print(',');
        SerialBT.print(ay_g,4); SerialBT.print(',');
        SerialBT.print(az_g,4); SerialBT.print(',');
        SerialBT.println(amag_g,4);
      }
    }
  }
}