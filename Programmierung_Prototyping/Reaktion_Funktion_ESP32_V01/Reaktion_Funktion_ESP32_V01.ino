// Functions ESC,ESP32,THW-1060, 
// Hardware: ESP32, THW-1060, MPU6050, 2S Lipo 2200 mAh

/*
Autor: Emrah Tekin
Date: 18.09.2025


*/

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>  
#include "esp32-hal-ledc.h"

#include <Arduino.h>
#include <ESP32Servo.h>



//******************MPU6050*************
#define MPU6050       0x68 // I2C adresse
#define ACCEL_CONFIG  0x1C //Hier stellst du die Empfindlichkeit des Beschleunigungssensors ein (±2g, ±4g, ±8g, ±16g)
#define GYRO_CONFIG   0x1B // 0x1B (GYRO_CONFIG) → Hier stellst du die Gyro-Empfindlichkeit ein (±250, ±500, ±1000, ±2000 °/s)
#define PWR_MGMT_1    0x6B //0x6B (PWR_MGMT_1) → Power-Management, z. B. Sleep-Mode ausschalten.
#define PWR_MGMT_2    0x6C // 0x6C (PWR_MGMT_2) → Weitere Power-Optionen (einzelne Sensorachsen aktivieren/deaktivieren)


//************Pin mapping***************
const int PIN_I2C_SDA  = 21; // ESP32 Default pin
const int PIN_I2C_SCL  = 22; // ESP32 default
const int PIN_ESC_SIGNAL = 5; // LEDC capable pin for ESC signal
const int PIN_BUZZ = 23; //// Buzzer (digital)
const int PIN_VBAT = 34; // Battery sense (ADC1, input-only)
const int PIN_SENSOR = 35;  // Analog angle sensor (ADC1, input-only)

//***************LEDC for ESC (servo signal)*****************
const int ESC_CH        = 0;    // LEDC channel
const int ESC_FREQ_HZ   = 50;   // 50 Hz servo signal ->PEriodendauer = 20 ms
const int ESC_RES_BITS  = 16;   // 16-bit resolution (0..65535)

//**********Control & sensor config***************
#define Gyro_amount   0.0996f  //Gewichtungsfaktor für den Komplementärfilter
const uint8_t accSens   = 0;  // 0=2g,1=4g,2=8g,3=16g
const uint8_t gyroSens  = 1;  // 0=250,1=500,2=1000,3=2000 dps

//*******************Gains (state feedback-like)***********
float K1Gain  = 175.0f; // Verstärkung für den WINKEL. Gross gewählt, weil die Hauptaufgabe ist, den Roboter wieder aufzurichten. 
float K2Gain  = 16.0f;  // Verstärkung für die Drehgeschwindigkeit (Gyro). 
float K3Gain  = 0.04f;  // Verstärkung für die Motorposition / integrierte Geschwindigkeit. "Sehr Klein, sorgt für langsame Korrektur lanfristiger Driffts."
float K4Gain  = 13.0f;  //Verstärkung für die Radgeschwindigkeit. Mittelhoch, um schnelle Bewegungen auszugleichen, aber kleiner als K1.

//****************Main loop step [ms]*****************
float loop_time = 8.0f; // das entspricht einer Abtastfrequenz von ca. 125 Hz.
//Wir Wollen den Regler schnell genug laufen lassen, damit der Roboter stabil bleibt.

//*****************Runtime vars******************
int pwmCMD = 0;   // aktueller PWM-Steuerwert für den ESC
int32_t motor_speed = 0; // integrierte Geschwindigkeit / "Motorposition"
float motor_speed_enc = 0.0f;  // gemessene Motorgeschwindigkeit vom Encoder

int16_t AcX, AcY, AcXc, AcYc, GyZ, gyroZ; 
// AcX, AcY   → rohe Beschleunigungswerte vom MPU6050 (X- und Y-Achse)
// AcXc, AcYc → kalibrierte Beschleunigungswerte (Offset korrigiert)
// GyZ        → roher Gyroskopwert (Z-Achse)
// gyroZ      → skalierter Gyroskopwert in °/s (abhängig von gewählter Empfindlichkeit)

float angle_prev = 0.0f;    // letzter gemessener Winkel (rad), für Differenzbildung
float velocity = 0.0f;      // berechnete Winkelgeschwindigkeit (rad/s)
long vel_angle_prev_ts = 0; // Zeitstempel (µs) der letzten Geschwindigkeitsberechnung
float vel_angle_prev = 0.0f; // Winkel beim letzten Geschwindigkeits-Update
int32_t full_rotations = 0;   // Anzahl kompletter Umdrehungen (Überlaufkorrektur)
int32_t vel_full_rotations = 0;   // vorheriger Umdrehungszähler für die Geschwindigkeit

//**********ESP32 ADC range***********
int max_raw_count = 4095; // Maximalwert des 12 - Bit ADC
int min_raw_count = 0;   // Minimalwert des ADC

// Calibration offsets structure
struct AccOffsetsObj {
  int     ID; // Kennung(zb 78 = gültige Kalibrierung)
  int16_t X;  // Offset für Beschleunigungssensor X-Achse
  int16_t Y;  // Offset für Beschleunigungssensor Y - Achse
};

AccOffsetsObj offsets; // Objekt, in dem die Werte gespeichert werden. 

int16_t GyZ_offset = 0;  // Gyroskop-Z-Offset (nach Kalibrierung)
int32_t GyZ_offset_sum = 0; // Zwischensumme während der Offset-Berechnung

float alpha = 0.40f;    // Filterkoeffizient für Tiefpass (Gyro-Glättung)
float gyroZfilt = 0.0f; // gefilterter Gyroskopwert (Z-Achse)
//alpha, gyroZfilt → Parameter für einen Tiefpassfilter: gyroZfilt = alpha*gyroZ + (1-alpha)*gyroZfilt. Glättet die Gyro-Messung.
float robot_angle = 0.0f;  // berechneter Neigungswinkel des Roboters (aus Gyro + Acc)
float Acc_angle = 0.0f;   // Winkel aus Accelerometer (arctan2)
bool vertical = false;    // Statusflag: Roboter im vertikalen Bereich?
bool calibrating = false; // Flag: Kalibrierung läuft gerade
bool calibrated = false;  // Flag: Kalibrierung erfolgreich abgeschlossen

//*****************Baterry scaling *************
float VBAT_SCALE = 0.00400f;  // <-- PLACEHOLDER. Calibrate for your divider & ADC.

// *********Wenn nötig ist Buzz************
static inline void beeOK(){
  digitalWrite(PIN_BUZZ, HIGH); delay(70);
  digitalWrite(PIN_BUZZ, LOW); delay(80);
  digitalWrite(PIN_BUZZ, HIGH); delay(70);
  digitalWrite(PIN_BUZZ, LOW); 
}

// Map microseconds 1000.. 2000 to LEDC duty at 50 Hz, 16-bit resolution
void escWriteMicroseconds(uint16_t us){
  const uint32_t period_us = 2000UL;   // 50 Hz = 20 ms
  const uint32_t maxDuty = (1UL << ESC_RES_BITS) - 1UL; // 65535 // 1UL << ESC bedeutet 16 bit links verscheiben
  if(us < 500) us = 500; // safety clamp
  if(us > 2500) us = 2500;
  uint32_t duty = (uint32_t)((uint64_t)us*maxDuty / period_us);
  // Berechnet Duty-Cycle aus Pulsbreite (us).
  // Formel: duty = (us / period_us) * maxDuty
  // (64 Bit Multiplikation zur Vermeidung von Überlauf)

  ledcWrite(ESC_CH, duty);
}

// ESC arming at neutral
void escArmNeutral(uint16_t neutral_us = 1500, uint16_t ms_hold = 2000){
  escWriteMicroseconds(neutral_us);
  delay(ms_hold);
}

//Low-level I2C write
void writeTo(uint8_t dev, uint8_t addr, uint8_t val){
  //dev → I²C-Geräteadresse (z. B. 0x68 für MPU6050)
  Wire.beginTransmission(dev); // I2C-Übertragung an Gerät mit Adresse 'dev' starten
  // addr → Registeradresse im Gerät (z. B. PWR_MGMT_1 = 0x6B)
  Wire.write(addr);           // Register-Adresse schicken
  //val → Wert, der in dieses Register geschrieben werden soll
  Wire.write(val);            // Wert 'val' an dieses Register schreiben
  Wire.endTransmission(true); // Übertragung beenden und Daten senden
}

//**************IMU init & gyro bias *****************
void angle_setup(){
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  delay(100);

  writeTo(MPU6050, PWR_MGMT_1, 0); // wake up  // Sleep aus, interner Takt (CLKSEL=0)
  // ACCEL_CONFIG = accSens<<3 wählt ±2/4/8/16 g, GYRO_CONFIG = gyroSens<<3 wählt ±250/500/1000/2000 °/s (FS_SEL Bits)
  writeTo(MPU6050, ACCEL_CONFIG, (accSens << 3)); // accel FS // ACCEL_FS_SEL Bits [4:3] nach Datenblatt
  writeTo(MPU6050, GYRO_CONFIG, (gyroSens << 3)); // gyro FS // GYRO_FS_SEL  Bits [4:3]
  delay(100);

  GyZ_offset_sum = 0;
  for (int i=0; i < 1024; i++){
    //quick read for bias estimation
    Wire.beginTransmission(MPU6050);
    Wire.write(0x47); // GYRO_ZOUT_H // Registerpointer auf GYRO_ZOUT_H (0x47) setzen
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, (uint8_t)2, (uint8_t)true);
    int16_t gz = (Wire.read() << 8) | Wire.read();
    GyZ_offset_sum += gz;
    delay(5);

  }
  GyZ_offset = (int16_t)(GyZ_offset_sum >> 10);

  beeOK();
  Serial.print("GyZ offset value = ");
  Serial.println(GyZ_offset);

}

//****************Read IMU + complementary filter***********
void angle_calc(){
  // Acc X, Y
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, (uint8_t)4, (uint8_t)true);
  AcX = (Wire.read() << 8 | Wire.read());
  AcY = (Wire.read() << 8 | Wire.read());

  //Gyro Z
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050,(uint8_t)2, (uint8_t)true);
  GyZ = (Wire.read() << 8 | Wire.read());

  // offsets & fusion 
  AcXc = AcX - offsets.X;
  AcYc = AcY - offsets.Y;
  GyZ =  GyZ - GyZ_offset;

  // integrate gyro (deg)
  robot_angle += ((float)GyZ * (loop_time / 1000.0f) / 65.536f);
  // accel angle (deg)
  Acc_angle = atan2((float)AcYc, (float)-AcXc) * 57.2958f;
  // complementary filter
  robot_angle = robot_angle * Gyro_amount + Acc_angle*(1.0f - Gyro_amount);

  // vertical window with hysteresis
  if(fabsf(robot_angle) > 6.0f) vertical = false;
  if(fabsf(robot_angle) < 0.3f) vertical = true;
  
}

//***************Baterry helpers (2S Lipo)*********
float readBatteryVoltage(){
  // Simple linear scale approach - CALIBRATE VBAT_SCALE!
   int raw = analogRead(PIN_VBAT);
   return raw * VBAT_SCALE;
}

void battVoltageCheck(){
  // Typical 2S thresholds (tune for your cells/ESC);
  const float WARN_V    = 7.50f;  //3.60 V / cell
  const float CRITICAL_V = 6.60f; // 3.30 V / cell
  float v = readBatteryVoltage();

  // Simple beeper policy;
  if(v <= CRITICAL_V){
    digitalWrite(PIN_BUZZ, HIGH); delay(100);
    digitalWrite(PIN_BUZZ, LOW);  delay(100);
  }else if (v <= WARN_V) {
    // slow beep on warn
    digitalWrite(PIN_BUZZ, HIGH); delay(60);
    digitalWrite(PIN_BUZZ, LOW);
  } else {
    digitalWrite(PIN_BUZZ, LOW);
  }
  // Optional debug:
  Serial.print("VBAT[V] = "); Serial.println(v, 2);
}

// ******************Motor control cia ESC (map -255...2555 to 1000...2000 us)****************

void Motor_control(int pwm){
  //deadband to avoid jitter
  const int dead = 12;
  if(abs(pwm) < dead) pwm = 0;

  // Map to us around 1500
  int us = 1500 + (int)((float)pwm * (500.0f / 255.0f)); // +/- 500 us span
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;

  escWriteMicroseconds((uint16_t)us);

}

// ------------ Serial live tuning ------------
void printValues() {
  Serial.print("K1: "); Serial.print(K1Gain);
  Serial.print(" K2: "); Serial.print(K2Gain);
  Serial.print(" K3: "); Serial.print(K3Gain, 3);
  Serial.print(" K4: "); Serial.println(K4Gain);
}


 // Print a one-line help and a prompt. Keep it short.
void tuningPromptOnce() {
  static bool shown = false;
  if (shown) return;
  Serial.println("Tuning: p+/p- (K1), i+/i- (K2), s+/s- (K4), a+/a- (K3), c+/c- (cal)");
  Serial.print("> ");               // simple prompt
  shown = true;
}

int Tuning() {
  if (!Serial.available()) return 0;
  delay(2);
  char param = Serial.read();
  if (!Serial.available()) return 0;
  char cmd = Serial.read();
  Serial.flush();
  // p -> K1 (proportinale gain)
  // i -> K2 (integral/gyro gain)
  // s -> K4 (speed gain)
  // a - > K3(angle correction)
  // c -> calibration mod
  switch (param) {
    case 'i':
      if (cmd == '+') K2Gain += 0.5f;
      if (cmd == '-') K2Gain -= 0.5f;
      printValues(); break;
    case 'p':
      if (cmd == '+') K1Gain += 1.0f;
      if (cmd == '-') K1Gain -= 1.0f;
      printValues(); break;
    case 's':
      if (cmd == '+') K4Gain += 1.0f;
      if (cmd == '-') K4Gain -= 1.0f;
      printValues(); break;
    case 'a':
      if (cmd == '+') K3Gain += 0.005f;
      if (cmd == '-') K3Gain -= 0.005f;
      printValues(); break;
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating) {
        Serial.println("calibrating off");
        Serial.print("X: "); Serial.print(AcX + 16384);
        Serial.print(" Y: "); Serial.println(AcY);
        if (abs(AcY) < 3000) {
          offsets.ID = 78;
          offsets.X  = AcX + 16384;
          offsets.Y  = AcY;
          digitalWrite(PIN_BUZZ, HIGH); delay(70);
          digitalWrite(PIN_BUZZ, LOW);
          EEPROM.put(0, offsets);
          EEPROM.commit();        // important on ESP32
          calibrating = false;
          calibrated  = true;
        } else {
          Serial.println("The angle are wrong!!!");
          calibrating = false;
          digitalWrite(PIN_BUZZ, HIGH); delay(50);
          digitalWrite(PIN_BUZZ, LOW);  delay(70);
          digitalWrite(PIN_BUZZ, HIGH); delay(50);
          digitalWrite(PIN_BUZZ, LOW);
        }
      }
      break;
  }
   
  return 1;
}

// ------------ Angle sensor (analog potentiometer-like) ------------
#define _2PI 6.28318530718f
float getSensorAngle() {
  int raw = analogRead(PIN_SENSOR);
  int cpr = (max_raw_count - min_raw_count);
  if (cpr <= 0) cpr = 1;
  return ((float)(raw - min_raw_count) / (float)cpr) * _2PI;
}

// Angular velocity [rad/s] from successive angle reads
float getVelocity() {
  float val = getSensorAngle();
  long ts   = micros();
  float d_angle = val - vel_angle_prev;

  // unwrap across 0..2π
  if (fabsf(d_angle) > (0.8f * _2PI)) full_rotations += (d_angle > 0) ? -1 : 1;

  float Ts = (ts - vel_angle_prev_ts) * 1e-6f;
  if (Ts <= 0.0f) Ts = 1e-6f;

  velocity = ((float)(full_rotations - vel_full_rotations) * _2PI
              + (val - vel_angle_prev)) / Ts;

  vel_angle_prev     = val;
  vel_full_rotations = full_rotations;
  vel_angle_prev_ts  = ts;

  return velocity;
}

