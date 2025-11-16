# SlinkySpace Start  
### *Multi-Sensor Demonstration Platform – ESP32 + BMP280 + SS49E + Battery Monitor*

---

## 🚀 Projekt áttekintése

A **SlincSpace Start** egy oktatási és fejlesztési célú, moduláris szenzorplatform  
**Wemos Lolin32 Lite (ESP32)** mikrokontrolleren.

A projekt célja:

- különböző érzékelők működésének demonstrálása  
- telemetriai mérőrendszerek alapjainak megértése  
- CanSat 2026 fejlesztési előkészítés  
- alkalmazott fizika + elektronika + programozás gyakorlása diákokkal  
- élő mérési példák megjelenítése (Serial Monitor / Plotter)

A rendszer alkalmas:

- hőmérséklet és légnyomás mérésére (BMP280)  
- mágneses tér erősségének és polaritásának mérésére (SS49E lineáris Hall)  
- akkufeszültség monitorozására (100k–100k osztóval)  
- nem blokkoló időzített mintavételezésre  
- mintavételi visszajelzésre (LED villanás)  

---

## 🧱 Hardver komponensek

### **Alaplap**
- Wemos Lolin32 Lite (ESP32, 3.3V)

### **Szenzorok**
- BMP280 – hőmérséklet + légnyomás (I²C)
- SS49E – lineáris Hall-effektus szenzor (analóg)
- (opcionális) HX711 + erőmérő cella
- (opcionális) LoRa modul (SX1276 / WLR089U0)

### **Kiegészítők**
- 100kΩ + 100kΩ feszültségosztó az akkumulátor méréséhez
- LED visszajelzés mintavételkor (beépített LED, GPIO2)

---

## 🔌 Bekötési táblázat

### **BMP280 (I²C)**

| BMP280 láb | ESP32 |
|------------|--------|
| VIN        | 3.3V   |
| GND        | GND    |
| SCL        | 23     |
| SDA        | 19     |

### **SS49E lineáris Hall-szenzor**

| SS49E láb | ESP32 |
|-----------|--------|
| VCC       | 3.3V   |
| GND       | GND    |
| OUT       | 34 (ADC1) |

### **Akkumulátor mérés (2:1 osztó)**

| Feszültségosztó közepe | ESP32 |
|------------------------|--------|
| 100k / 100k közép pont | 35 (ADC1) |

### **LED**
- Beépített LED → **GPIO2 (aktív LOW)**

---

## 📂 Könyvtárstruktúra (ajánlott)

