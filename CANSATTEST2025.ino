#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// -----------------------------------------------------------------------------
// HARDVER BEÁLLÍTÁSOK
// -----------------------------------------------------------------------------
#define SDA_PIN   19
#define SCL_PIN   23
#define BMP_ADDR  0x76

#define BAT_PIN   35     // Akkufeszültség ADC (100k-100k osztó)
#define HALL_PIN  34     // SS49E lineáris Hall-szenzor ADC bemenet

const float VREF = 3.3;       // ESP32 ADC referenciafeszültség
const int ADC_MAX = 4095;     // 12 bites ADC maximum
const float DIVIDER_RATIO = 2.0;  // 100k-100k osztó visszaszorzása

// -----------------------------------------------------------------------------
// GLOBÁLIS OBJEKTUMOK
// -----------------------------------------------------------------------------
Adafruit_BMP280 bmp;
bool bmp_ok = false;

unsigned long lastMeasure = 0;
const unsigned long MEASURE_INTERVAL = 1000;   // 1 másodperc

// -----------------------------------------------------------------------------
// Akkumulátor feszültség mérése (átlagolva)
// -----------------------------------------------------------------------------
float readBatteryVoltage() {
  long sum = 0;
  const int samples = 10;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(BAT_PIN);
    delay(2);
  }

  float adc_avg = sum / (float)samples;
  float voltage_adc = adc_avg * VREF / ADC_MAX;

  return voltage_adc * DIVIDER_RATIO;   // visszaszorzás az osztó miatt
}

// -----------------------------------------------------------------------------
// Hall szenzor (SS49E) mérése – analóg, lineáris
// -----------------------------------------------------------------------------
float readHallVoltage() {
  long sum = 0;
  const int samples = 10;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(HALL_PIN);
    delay(2);
  }

  float adc_avg = sum / (float)samples;
  return adc_avg * VREF / ADC_MAX;   // közvetlen feszültség az OUT lábon
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);//Beépített LED teszteléshez
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  while (!Serial);

  Serial.println();
  Serial.println("=== BMP280 + Akku + SS49E Hall Mérőrendszer ===");

  // I2C indítása
  Wire.begin(SDA_PIN, SCL_PIN);

  // BMP280 indítása
  if (!bmp.begin(BMP_ADDR)) {
    Serial.println("[BMP280] Nem található!");
  } else {
    bmp_ok = true;
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_63
    );
    Serial.println("[BMP280] OK.");
  }

  analogReadResolution(12);  // ESP32 ADC = 0..4095
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // --- 1 másodperces időzítő ---
  if (now - lastMeasure >= MEASURE_INTERVAL) {
    lastMeasure = now;

    // --- BMP280 ---
    float T = bmp_ok ? bmp.readTemperature() : NAN;
    float P = bmp_ok ? (bmp.readPressure() / 100.0) : NAN;

    // --- Akkumulátor feszültség ---
    float Ubat = readBatteryVoltage();

    // --- SS49E analóg Hall feszültség ---
    float Uhall = readHallVoltage();

    // LED villantás mintavétel jelzésére (nem blokkoló)
        
        digitalWrite(LED_BUILTIN, LOW);   // LED ON (aktív LOW)
        delay(30);                        // 30 ms → emberi szem is látja
        digitalWrite(LED_BUILTIN, HIGH);  // LED OFF

    // --- KIÍRÁS ---
    Serial.println("-------------------------------------------------");

    if (bmp_ok)
      Serial.printf("BMP280:    T = %.2f °C    P = %.1f hPa\n", T, P);
    else
      Serial.println("BMP280:    NEM mérhető");

    Serial.printf("Akkumulátor:   %.2f V\n", Ubat);
    Serial.printf("Hall (SS49E):  %.3f V  ", Uhall);

    // Polaritás értelmezése
    if (Uhall > 1.70)
      Serial.println("(Északi pólus → közelít)");
    else if (Uhall < 1.55)
      Serial.println("(Déli pólus → közelít)");
    else
      Serial.println("(Nincs mágnes)");

  }
}
