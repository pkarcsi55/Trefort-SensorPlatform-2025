# CANSAT 2026 – ESP32 (Wemos Lolin32 Lite) teljes pinout

> Kód alapján ellenőrzött végleges kiosztás  
> microSD és LoRa közös VSPI buszon működik.  
> GPS UART2 bekötés: GPS TX → ESP32 GPIO17, GPS RX ← ESP32 GPIO16.

---

## Pinout táblázat

| Funkció | Modul / eszköz | Jel | ESP32 GPIO | Megjegyzés |
|---|---|---|---|---|
| I2C SDA | BMP280 / BME280 | SDA | GPIO32 | I2C busz |
| I2C SCL | BMP280 / BME280 | SCL | GPIO33 | I2C busz |
| HX711 adat | HX711 | DOUT | GPIO22 | digitális input |
| HX711 óra | HX711 | SCK | GPIO27 | digitális output |
| Külső hőmérő | NTC termisztor | OUT | GPIO13 | ADC bemenet |
| Hall szenzor | Hall | OUT | GPIO34 | ADC bemenet, input only |
| Akkufeszültség | feszültségosztó | BAT | GPIO35 | ADC bemenet, input only |
| GPS TX → ESP RX2 | NEO-8M | TX | GPIO17 | UART2 RX |
| GPS RX ← ESP TX2 | NEO-8M | RX | GPIO16 | UART2 TX |
| microSD CS | MicroSD | CS | GPIO5 | VSPI CS |
| microSD SCK | MicroSD | SCK | GPIO18 | közös VSPI SCK |
| microSD MOSI | MicroSD | MOSI | GPIO19 | közös VSPI MOSI |
| microSD MISO | MicroSD | MISO | GPIO23 | közös VSPI MISO |
| LoRa SCK | RFM95 | SCK | GPIO18 | közös VSPI SCK |
| LoRa MOSI | RFM95 | MOSI | GPIO19 | közös VSPI MOSI |
| LoRa MISO | RFM95 | MISO | GPIO23 | közös VSPI MISO |
| LoRa CS / NSS | RFM95 | NSS | GPIO25 | külön chip select |
| LoRa IRQ | RFM95 | DIO0 | GPIO26 | RxDone / TxDone megszakítás |
| LoRa Reset | RFM95 | RST | GPIO14 | LoRa modul reset |
| Táp | minden modul | VCC | 3V3 | csak 3,3 V |
| Föld | minden modul | GND | GND | közös föld |

---

## SPI busz megosztás

A microSD kártya és a LoRa RFM95 modul ugyanazt a VSPI buszt használja.

Közös SPI vonalak:

| SPI jel | ESP32 GPIO |
|---|---|
| SCK | GPIO18 |
| MOSI | GPIO19 |
| MISO | GPIO23 |

Külön chip select vonalak:

| Eszköz | CS / NSS GPIO |
|---|---|
| microSD | GPIO5 |
| LoRa RFM95 | GPIO25 |

A közös SPI busz miatt egyszerre mindig csak az egyik SPI-eszköz lehet kiválasztva. Ezt a külön CS/NSS vonalak biztosítják.

---

## GPS bekötés

A működő kód alapján a GPS UART2 bekötése:

| GPS modul | ESP32 |
|---|---|
| GPS TX | GPIO17, ESP32 RX2 |
| GPS RX | GPIO16, ESP32 TX2 |
| GND | GND |
| VCC | 3V3 |

Fontos: a soros kommunikációnál a TX és RX keresztben kapcsolódik.  
Ezért a GPS TX lába az ESP32 RX bemenetére megy, a GPS RX lába pedig az ESP32 TX kimenetére.

---

## Arduino pin define blokk

```cpp
// I2C
#define I2C_SDA    32
#define I2C_SCL    33

// HX711
#define HX_DOUT    22
#define HX_SCK     27

// Analog inputs
#define BAT_ADC_PIN   35
#define NTC_ADC_PIN   13
#define HALL_ADC_PIN  34

// GPS (UART2)
#define GPS_RX     17   // ESP32 RX2, ide jön a GPS TX
#define GPS_TX     16   // ESP32 TX2, innen megy a GPS RX
#define GPS_BAUD   9600

// VSPI / LoRa SPI
#define LORA_SCK   18
#define LORA_MISO  23
#define LORA_MOSI  19

// microSD
#define SD_CS      5

// LoRa (RFM95)
#define LORA_CS    25
#define LORA_RST   14
#define LORA_DIO0  26
