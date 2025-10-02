
Autor:Emrah Tekin
Date: 19.09.2025

# ESP32 Reaktionsrad-Balancer auf Seil  
**Mit THW-1060 ESC, MPU6050 (HW-123), RS-540, 2S LiPo**  

## Hardware
- **MCU:** FireBeetle ESP32 (DFR0478)  
- **Sensor:** MPU6050 IMU (HW-123, I²C, Adresse 0x68)  
- **ESC:** THW-1060 Regler (RC-Standard, 50 Hz, 1000–2000 µs)  
- **Motor:** RS-540PH-7021  
- **Akku:** 2S LiPo (z. B. 2200 mAh)  
- // TO DO ->**Buzzer:** Signalisierung (Arming, Kalibrierung, Warnung)  
- **VBAT-Messung:** Spannungsteiler → ESP32 ADC  

**Wichtige Hinweise:**  
- ESC-Signalpin muss ein **LEDC-fähiger GPIO** des ESP32 sein.  
- **GND** des ESC, Akkus und ESP32 muss verbunden sein.  
- MPU6050 benötigt **3.3 V Versorgung**.  


## Überblick
Dieses Projekt realisiert ein **Balance-System auf einem dünnen Seil** mit Hilfe eines **Reaktionsrades**.  
Ein **ESP32 FireBeetle** liest die **IMU MPU6050** über I²C aus, fusioniert Gyroskop- und Beschleunigungsdaten mittels 
**Komplementärfilter** und steuert das **Reaktionsrad** über einen **THW-1060 ESC** an.  

Die Stellgröße basiert auf einer **zustandsrückführungs-ähnlichen Regelung (LQR-inspiriert)** mit den Verstärkungen `K1..K4`. 

LQR-Regelung im Projekt

Dieses Projekt basiert auf dem in der Theorie-PDF „Linear Control of the Flywheel“ beschriebenen Ansatz 
(vgl. Kapitel 4.1 State feedback stabilizing controller).



***************************Regelgesetz:**************************

Das eingesetzte Steuerungsgesetz ist eine Zustandsrückführung nach dem LQR-Prinzip:

        u=−Kx

Im Code wird die Stellgröße (pwm) folgendermaßen berechnet:

pwm = K1 * φ + K2 * φ_dot + K4 * ω_enc + K3 * motor_speed

Zustandsgrößen

φ (robot_angle) → Neigungswinkel des Roboters

φ_dot (gyroZfilt) → Winkelgeschwindigkeit (Gyroskop, gefiltert)

ω_enc (motor_speed_enc) → gemessene Geschwindigkeit des Reaktionsrades

motor_speed → integrierte Geschwindigkeit (∫ ω_enc dt), entspricht einem Positionsanteil

*************************Bedeutung der Gains*********************

K1Gain = 175.0 → starker Rückstellfaktor für den Winkel φ

K2Gain = 16.0 → Dämpfung über Gyroskop, reduziert Überschwingen

K4Gain = 13.0 → Einfluss der Radgeschwindigkeit (schnelle Korrekturreaktionen)

K3Gain = 0.04 → kleiner Integralanteil, korrigiert langfristigen Drift

*********************************Verbindung zur Theorie*************************

In der Theorie-PDF wird hergeleitet, dass u = -Kx optimal gewählt wird, indem eine Riccati-Gleichung gelöst wird.

Die Matrix K hängt von den Gewichtungen Q (Zustände) und R (Energieaufwand des Motors) ab.

Die im Code eingesetzten Werte wurden praktisch abgestimmt, entsprechen aber konzeptionell der im PDF vorgestellten LQR-Struktur.

***********************
## Softwarearchitektur

### Funktionsdatei
- `writeTo()` → Low-Level I²C-Schreiben für MPU6050 Register  
- `angle_setup()` → Initialisierung IMU, Bias-Kompensation Gyro Z (1024 Samples)  
- `angle_calc()` → Liest AcX/AcY/GyZ, kompensiert Offsets, integriert Gyro, berechnet Acc-Winkel, kombiniert über Komplementärfilter  
- `escWriteMicroseconds()` → Mappt 1000–2000 µs Servo-Signal auf **LEDC Duty**  
- `escArmNeutral()` → ESC-Arming im Neutralpunkt (1500 µs, 2 s halten)  
- `Motor_control(pwm)` → Skaliert Bereich −255..255 auf 1000–2000 µs, inkl. Deadband  
- `Tuning()` → Einstellen der Gains `K1..K4` via Serial (`p/i/s/a` +/−), Kalibrierung `c+/c−`  
- `beepOK()` → Akustisches Feedback über Buzzer  
- `battVoltage()` → LiPo Überwachung (2S/3S Spannungsfenster)  

### Hauptprogramm
- `setup()` → Initialisierung, Offset-Laden (EEPROM/NVS), Pins, ESC-Arming, `angle_setup()`  
- `loop()` →  
  - alle **8 ms**: Sensor lesen, Filter anwenden, PWM berechnen, Motor ansteuern  
  - alle **1000 ms**: Batteriespannung prüfen, Kalibrierhinweise  

---

## Sensorfusion & Formeln

**Gyro-Integration (Winkel in °):**
\[
\varphi(t+\Delta t) = \varphi(t) + \frac{GyZ}{\text{Sensitivity}} \cdot \Delta t
\]

- Sensitivität Gyro je nach Bereich: ±500 °/s → 65.5 LSB/(°/s)  
- \(\Delta t = \text{loop\_time}/1000\)  

**Acc-Winkel (°):**
\[
\varphi_{Acc} = \arctan2(AcY_c,\,-AcX_c) \cdot \frac{180}{\pi}
\]

**Komplementärfilter:**
\[
\varphi = \alpha \cdot \varphi_{Gyro} + (1-\alpha) \cdot \varphi_{Acc}
\]

- \(\alpha = 0.98 \dots 0.996\)  

---

## Regelung

**Steuergesetz (LQR-inspiriert):**
\[
u = K_1 \cdot \varphi + K_2 \cdot \dot\varphi + K_4 \cdot \omega_{Rad} + K_3 \cdot \int \omega_{Rad} dt
\]

- \(\varphi\) = Neigungswinkel (complementary filter)  
- \(\dot\varphi\) = Winkelgeschwindigkeit (Gyro, gefiltert)  
- \(\omega_{Rad}\) = Raddrehzahl (falls Sensor vorhanden, sonst Schätzung)  
- \(\int \omega dt\) = integrierte Raddrehzahl  

---

## Parameterwahl
- `accSens = 0` → ±2 g (hohe Empfindlichkeit, 16384 LSB/g)  
- `gyroSens = 1` → ±500 °/s (Balanceanwendungen typisch, 65.5 LSB/°/s)  
- `Gyro_amount ≈ 0.996` → Gewichtung im Komplementärfilter  
- `loop_time = 8 ms` → Regelzyklusfrequenz ca. 125 Hz  

---

## Besonderheiten

- **Vertical Window mit Hysterese:**  
  ```c
  if (fabs(robot_angle) > 6.0) vertical = false;
  if (fabs(robot_angle) < 0.3) vertical = true;
  ```
  → Verhindert Flattern durch Sensorauschen (Schmitt-Trigger-Effekt).  

- **ESC-Steuerung (LEDC):**  
  - Kanal 0, 50 Hz, 16 Bit (0–65535 Duty)  
  - 1000–2000 µs = Vollrückwärts bis Vollvorwärts  

- **EEPROM/NVS:** Speichern von Kalibrier-Offsets (Acc X/Y, Gyro Z).  

---

## Sicherheit
- ESC verlangt beim Start **2 s neutrales Signal (1500 µs)**.  
- LiPo nur im sicheren Bereich betreiben (2S: 6.4–8.4 V).  
- Mechanik sichern (Rad kann plötzlich beschleunigen).  

---

## Grenzen
- Ohne Radsensor (`ω_enc`) sind die Terme `K3/K4` nur Schätzungen → vorsichtig einstellen.  
- Besser: Hall-Sensor oder Magnetencoder (AS5600) am Rad.  
- Komplementärfilter ist einfach; für höhere Präzision → Kalman-Filter.  

---

## Erste Schritte
1. Hardware verkabeln, GND verbinden.  
2. GPIOs in Code anpassen (`PIN_I2C_SDA/SCL`, `PIN_ESC_SIGNAL`).  
3. ESC arming durchführen (beim Start 2 s neutral).  
4. Seriellen Monitor öffnen (115200).  
5. Kalibrierung mit `c+` starten, System senkrecht halten, mit `c-` speichern.  
6. Gains `K1..K4` über Serial feinjustieren.  

---

## Referenzen
- Olivares & Albertos: *Linear control of the flywheel inverted pendulum*  
- DFRobot FireBeetle ESP32 Datenblatt  
- RS-540PH-7021 Motordaten  
- THW-1060 ESC Manual  
- FHGR Prototyping Aufgabenblatt  




