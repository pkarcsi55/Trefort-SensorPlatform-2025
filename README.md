# SlinkySpace Start  
### *Multi-Sensor Demonstration Platform – ESP32 + BMP280 + SS49E + Battery Monitor*

---

## 🚀 Projekt áttekintése

A **SlinkySpace Start** egy oktatási és fejlesztési célú, moduláris szenzorplatform  
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
- BME280 – hőmérséklet + légnyomás (I²C)
- SS49E – lineáris Hall-effektus szenzor (analóg)
- HX711 + erőmérő cella
  

### **Kiegészítők**
- Micro SD https://www.hestore.hu/prod_10042087.html
- LoRa modul RFM95W-868S2 https://www.hestore.hu/prod_10040543.html
- Antenna https://www.hestore.hu/prod_10041671.html
- Antenna csatlakozó https://www.hestore.hu/prod_10041671.html
- Áramforrás Li-Ion akkumulátor, 18650, 3.7V, 3400mAh INR18650-35E (SAMSUNG) https://www.hestore.hu/prod_10049687.html
- 100kΩ + 100kΩ feszültségosztó az akkumulátor méréséhez
- LED visszajelzés mintavételkor (beépített LED, GPIO2)



