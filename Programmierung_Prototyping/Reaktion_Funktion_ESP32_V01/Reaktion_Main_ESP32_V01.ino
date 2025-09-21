/*
Autor: Emrah Tekin
Date:  19.09.2025

*/

#include <Wire.h>
#include <EEPROM.h>

// Forward declarations from functions_esc.ino are not strictly needed in .ino sketches,
// but we keep prototypes here for clarity when navigating.
void angle_setup();
void angle_calc();
void Motor_control(int pwm);
int  Tuning();
void escArmNeutral(uint16_t neutral_us = 1500, uint16_t ms_hold = 2000);
float getVelocity();
void battVoltageCheck();

// Globals defined in functions_esc.ino (Arduino concatenates .ino tabs → shared)
extern float alpha, gyroZfilt, robot_angle, loop_time, motor_speed_enc;
extern int16_t GyZ, gyroZ;
extern int32_t motor_speed;
extern bool vertical, calibrating, calibrated;

extern const int PIN_ESC_SIGNAL, PIN_BUZZ, PIN_VBAT, PIN_SENSOR;
extern const int ESC_CH, ESC_FREQ_HZ, ESC_RES_BITS;

void setup() {
  Serial.begin(115200);

 // pinMode(PIN_BUZZ, OUTPUT);
 // digitalWrite(PIN_BUZZ, LOW);

  // ESC output: 50 Hz / 16-bit
  ledcSetup(ESC_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  ledcAttachPin(PIN_ESC_SIGNAL, ESC_CH);
  // Arm ESC at neutral
  escArmNeutral(1500, 2000);

  // ADC config (optional fine tune)
  // analogReadResolution(12);       // default on ESP32 core
  // analogSetAttenuation(ADC_11db); // expand range if your divider is high

  // EEPROM for IMU offsets
  EEPROM.begin(64);
  // offsets & 'calibrated' are handled in functions_esc.ino during tuning

  delay(300);
  // Short beep
 // digitalWrite(PIN_BUZZ, HIGH); delay(70);
 // digitalWrite(PIN_BUZZ, LOW);

  // IMU init + gyro bias
  angle_setup();
}

void loop() {
  static uint32_t t_ctrl = 0;
  static uint32_t t_1s   = 0;

  uint32_t nowMs = millis();

  // ~loop_time [ms] control loop
  if (nowMs - t_ctrl >= (uint32_t)loop_time) {
    Tuning();
    angle_calc();

    if (vertical && calibrated && !calibrating) {
      // Gyro Z in dps (approx like original)
      gyroZ = GyZ / 131.0f;
      gyroZfilt = alpha * gyroZ + (1.0f - alpha) * gyroZfilt;

      // Reaction wheel speed from analog "encoder"
      motor_speed_enc = getVelocity();

      // Your original state feedback structure
      int pwm = (int)(
          175.0f * robot_angle     // K1
        + 16.0f  * gyroZfilt       // K2
        + 13.0f  * motor_speed_enc // K4
        + 0.04f  * motor_speed     // K3
      );

      pwm = constrain(pwm, -255, 255);
      Motor_control(pwm);

      motor_speed += (int32_t)motor_speed_enc;
    } else {
      Motor_control(0);
      motor_speed = 0;
    }

    t_ctrl = nowMs;
  }

  // Once per second: battery & messages
  if (nowMs - t_1s >= 1000) {
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing point... Use Bluetooth!!!"); // TO DO mit dem Bloutut modul
    }

    battVoltageCheck();   // uses calibrated readBatteryVoltage()

    t_1s = nowMs;
  }
}
