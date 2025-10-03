
# ESP32 Reaction Wheel Balancer — README

> **Kurz (DE):** Dieses Projekt balanciert ein Gerät auf einem Seil mittels **Reaktionsrad**. Der Code ist in **zwei Dateien** organisiert: `Reaktion_Main_ESP32_V01.ino` (Ablauf/Loop) und `functions_esc.ino` (Sensorik, ESC, Tuning, Utilities). **Serielle Tuning‑Kommandos** erlauben Live‑Anpassung (K1..K4), **Watchdog** schützt vor Hängern, **PC‑Beep** (ASCII BEL) signalisiert Ereignisse.  
>
> **Kısa (TR):** Bu proje **reaksiyon tekeri** ile bir yapıyı ip/çelik tel üstünde dengelemeyi amaçlar. Kod iki dosyada toplanır: `Reaktion_Main_ESP32_V01.ino` (ana akış) ve `functions_esc.ino` (sensör, ESC, tuning). **Seri port komutları** ile (K1..K4) canlı ayar yapılır, **watchdog** kilitlenmelere karşı korur, **PC bip sesi** olayları bildirir.

---

## 1) System Overview

- **MCU:** FireBeetle / ESP32 (LEDC PWM + I2C)
- **IMU:** MPU6050 (Accel + Gyro, I2C @ 400 kHz)
- **Actuator:** Brushed DC motor (RS‑540) über **THW‑1060 ESC**
- **Power:** 2S LiPo 2200 mAh (VBAT Messung über Spannungsteiler auf ADC1)
- **Mechanik:** Reaktionsrad (3D‑Druck), Lager, Halterung
- **Ziel:** Balancieren auf 1–2 mm flexiblem Draht/Seil; **Zustandsrückführung** (K1..K4) stabilisiert den Winkel.

### Kontrollidee (vereinfacht)

Reglerausgang `pwm` wird aus Zustandsgrößen gebildet:
```
pwm = K1 * angle  +  K2 * gyroZ_filtered  +  K4 * wheel_speed  +  K3 * wheel_position_integral
```
- **K1**: Winkel (starker Term, richtet auf)
- **K2**: Dämpfung (Gyro)
- **K4**: Raddrehzahl (vom analogen Winkelgeber abgeleitet)
- **K3**: Langsamer Drift‑Ausgleich (integrierte Raddrehzahl)

`pwm ∈ [−255, +255]` wird linear auf **ESC‑Servosignal** gemappt:  
`1000 µs` (rückwärts) — `1500 µs` (neutral) — `2000 µs` (vorwärts).

---

## 2) Repository Layout

```
/src
 ├─ Reaktion_Main_ESP32_V01.ino     # setup(), loop(), Watchdog, Zyklus
 └─ functions_esc.ino               # ESC, IMU, Battery, Tuning, Velocity
```

- **Main** hält sich schlank: ruft pro Zyklus **einmal** `Tuning()` und `angle_calc_step()` auf, setzt `Motor_control(pwm)`, füttert Watchdog, macht 1‑Sekunden‑Aufgaben (Batterie).
- **Functions** enthält alle Hardware‑Nahe Funktionen, Tuning, Debug‑Ausgaben und Hilfsroutinen.

---

## 3) Hardware & Wiring

| Modul         | Anschluss ESP32                     | Hinweise |
|---------------|-------------------------------------|----------|
| **MPU6050**   | SDA → GPIO21, SCL → GPIO22          | I2C @ 400 kHz. Versorgung 3.3V, GND. AD0=GND (0x68). |
| **ESC THW‑1060** | Signal → GPIO5 (LEDC), GND gemeinsam | 50 Hz Servo‑Signal. Versorgung Motor separat (2S LiPo). |
| **VBAT Divider** | Analog → GPIO34 (ADC1)            | Kalibriere `VBAT_SCALE`. |
| **Angle Sensor** | Analog → GPIO35 (ADC1)            | Potenziometer/AS5600‑Analog o. ä. für Raddrehung. |
| **Buzzer** (optional) | GPIO23                         | In SW ersetzt durch **PC‑Beep** (ASCII BEL). |

> **Masse (GND) muss gemeinsam** zwischen ESC/IMU/ESP32 verbunden sein. LiPo nicht direkt an ESP32 anschließen!

---

## 4) Build & Flash

1. **Arduino IDE** (oder PlatformIO) mit ESP32‑Boardpaket.  
2. Board: *ESP32 Dev Module* (oder spezifisch **FireBeetle**).  
3. Benötigte Libs: `ESP32Servo` (für bequeme `writeMicroseconds`), ansonsten nur Standard (`Wire`, `EEPROM`).  
4. `functions_esc.ino` und `Reaktion_Main_ESP32_V01.ino` **in ein Sketch‑Verzeichnis** legen (gleiche Ebene).  
5. **Seriell 115200 Baud**, *Neue Zeile/CRLF* aus, **und** Eingaben *ohne Enter* (z. B. `p+`).

---

## 5) Runtime Parameter & Tuning (Seriell)

### Live‑Kommandos (2‑Zeichen)

| Kommando | Wirkung                     | Schrittweite | Bereich (Clamp) |
|----------|-----------------------------|--------------|------------------|
| `p+`, `p-` | `K1` (angle) up/down        | `±1.0`       | `[0 … 500]`      |
| `i+`, `i-` | `K2` (gyro) up/down         | `±0.5`       | `[0 … 100]`      |
| `s+`, `s-` | `K4` (wheel speed) up/down  | `±1.0`       | `[0 … 200]`      |
| `a+`, `a-` | `K3` (integral) up/down     | `±0.005`     | `[0 … 1.0]`      |
| `c+`       | **Kalibrierung START**      | —            | —                |
| `c-`       | **Kalibrierung STOP/Speich.** | —          | —                |
| `m+`,`m-`  | IMU‑Ausgabe an/aus         | —            | —                |

- Erfolgreiche Aktionen bestätigen mit Text + **PC‑Beep** (ASCII 0x07).  
- Unbekannte Kommandos → Fehlerton + Kurz‑Hilfe.  
- Nach jeder Aktion erscheint ein **Prompt** `> `.

> **Hinweis:** `Tuning()` verarbeitet Befehle nur, wenn **min. 2 Zeichen** gleichzeitig eintreffen (z. B. `p+`).

### Empfohlene Startwerte
```
K1 = 175.0
K2 = 16.0
K4 = 13.0
K3 = 0.04
loop_time = 8 ms (≈125 Hz)
alpha = 0.40   # LF for gyroZ
Gyro_amount ≈ 0.10  # Complementary filter (gyro‑anteil)
```

---

## 6) Control Loop (Main)

- **Periodisch** alle `loop_time` Millisekunden (Default `8 ms`).  
- Reihenfolge im Zyklus:
  1) `tuned = Tuning();` (nicht‑blockierend, optional Log)  
  2) `imu_status = angle_calc_step();` (I²C lesen + Fusion + vertikal Fenster)  
  3) Wenn `vertical && calibrated && !calibrating` → **ACTIVE**:  
     - `gyroZfilt = alpha*gyroZ + (1-alpha)*gyroZfilt`  
     - `motor_speed_enc = getVelocity()` (vom Analogsensor, [rad/s], **LPF** in Funktion)  
     - `pwm` berechnen → clamp → `Motor_control(pwm)`  
     - Debug‑Kurzzeile alle 200 ms (PWM, Winkel, Gyro, w)  
     - `motor_speed += motor_speed_enc` (einfache Integration)  
  4) Sonst **IDLE**: `Motor_control(0)`; Integrator auf 0.  
  5) **Watchdog reset** am Ende des Slots.  
- **Einmal pro Sekunde:** Batterie prüfen, Kalibrierhinweis drucken.

---

## 7) IMU Processing & Debug

- I²C‑Reads mit **Plausibilitätsprüfung** (`requestFrom`‑Anzahl, `endTransmission` Codes).  
- Fusion: **Gyro integrieren** (deg), **Accel‑Winkel** aus `atan2`, dann **Komplementärfilter**.  
- Sicherungen: `isfinite`‑Check, **Clamp ±45°**, **Hysterese** für `vertical`.  
- **Status‑Bits**: `IMU_ERR_TX` | `IMU_ERR_RX` | `IMU_ERR_NAN` | `IMU_ERR_CLAMP`.  
- **Rate‑Limited Log** (200 ms): `IMU ang=… gyro=… acc=…`, über `m+/m-` aktivierbar.  

**Kalibrierung (`c+ / c-`)**  
- `c+` → Kalibrierung an (Gerät aufrecht halten).  
- `c-` → Offsets prüfen, **EEPROM speichern**, Erfolgston.  
- Winkel‑Fensterprüfung (`abs(AcY) < 3000`) gibt Sicherheit gegen falsche Lage.

---

## 8) Motor Control & ESC

- Deadband um PWM=0 (±12) → Zittern vermeiden.  
- Linear Map: `pwm (−255…+255) → us (1000…2000)`; **1500 µs neutral**.  
- LEDC 16‑Bit @ 50 Hz; alternativ `ESP32Servo` direkt `writeMicroseconds`.  
- **Arming**: `escArmNeutral(1500, 2000)` hält neutralen Puls für z. B. 2 s (mit Konsolen‑Beep).

---

## 9) Battery Monitoring

- `VBAT_SCALE` **kalibrieren** (Multimeter!).  
- Schwellwerte (2S): `WARN_V=7.50 V`, `CRITICAL_V=6.60 V` (anpassen).  
- Warnungen als Text + **PC‑Beep** (max. alle 10 s, um Spam zu vermeiden).

---

## 10) Watchdog (ESP32 Task WDT)

- Initialisierung in `setup()`:
  ```cpp
  esp_task_wdt_init(2, true);  // 2 s Timeout, panic=true
  esp_task_wdt_add(NULL);      // aktuelle (loop) Task
  ```
- **Reset** am Ende jedes Steuer‑Slots:
  ```cpp
  esp_task_wdt_reset();
  ```
- Längere Blöcke (Arming, langsame I²C‑Abschnitte) evtl. zusätzlich füttern oder in kurze Chunks teilen.

---

## 11) Debugging & Prompt

- **Prompt** `> ` zeigt: „Eingabe erwartet“.  
- Statuswechsel (ACTIVE/IDLE) nur **kantengetriggert** loggen → ruhige Konsole.  
- 200‑ms‑Kurzzeilen: kompakt `PWM, ang, gyro, w`.  
- Übermäßige Ausgaben vermeiden (Timing & USB‑Fluss).

---

## 12) Sicherheit & Hinweise

- **GND gemeinsam** (ESC↔ESP32↔IMU).  
- ESC/Motor getrennt versorgen; **LiPo Safety** (Kurzschluss, Polung!).  
- Seil/Schwungrad mechanisch sichern (Schutzbrille, Freiraum).  
- **Failsafe**: Bei `!vertical` oder `!calibrated` → Motor neutral.  
- Vor Start: `escArmNeutral()`; danach erst Tuning & Aktivierung.

---

## 13) Bekannte Grenzen / To‑Dos

- Analoger Winkelgeber kann rauschen: **LPF** ist enthalten (`beta≈0.2`), evtl. optimieren.  
- Kein absoluter Nullpunkt beim Rad → Integratordrift beachten (`K3` klein halten).  
- `VBAT_SCALE` Platzhalter → unbedingt kalibrieren.  
- `Tuning()` erwartet 2‑Zeichen‑Kommandos **ohne Enter**. Gerekirse satır bazlı parser eklenebilir.

---

## 14) Kurzanleitung (Erstbetrieb)

1. Verkabelung prüfen, LiPo anschließen, GND gemeinsam!  
2. Flashen, Seriell 115200 Baud öffnen.  
3. **Arming** läuft automatisch (1500 µs).  
4. `c+` → Gerät senkrecht halten → `c-` → Offsets gespeichert (Beep).  
5. `m+` (IMU Log an), `p+ / i+ / s+ / a+` mit kleinen Schritten tunen.  
6. Balance testen: `vertical` TRUE wird angezeigt; Regelung aktiviert sich.  
7. Batterie beobachten; bei Warnung laden/wechseln.

---

## 15) Lizenz & Autor

- **Autor:** Emrah Tekin (FHGR Prototyping, 2025)  
- **Begleitung:** „Fizik Pro“ (Mobilrobotik, ESP32, C/Arduino)
- **Lizenz:** MIT (optional anpassen)

