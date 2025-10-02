/*
Autor: Emrah Tekin
Date:  19.09.2025

*/

#include <Wire.h>
#include <EEPROM.h>
#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// Forward declarations from functions_esc.ino are not strictly needed in .ino sketches,
// but we keep prototypes here for clarity when navigating.
void angle_setup();
void angle_calc();
void Motor_control(int pwm);
int  Tuning();
void escArmNeutral(uint16_t neutral_us, uint16_t ms_hold );
float getVelocity();
void battVoltageCheck();
void tuningPromptOnce();

// Globals defined in functions_esc.ino (Arduino concatenates .ino tabs → shared)
extern float alpha, gyroZfilt, robot_angle, loop_time, motor_speed_enc;
extern int16_t GyZ, gyroZ;
extern int32_t motor_speed;
extern bool vertical, calibrating, calibrated;

extern const int PIN_ESC_SIGNAL, PIN_BUZZ, PIN_VBAT, PIN_SENSOR;
extern const int ESC_CH, ESC_FREQ_HZ, ESC_RES_BITS;

Servo esc;

// ---- DEBUG flags/vars (C-style) ----
#define DEBUG_ON         1
#define DBG_BRIEF_MS     200

static uint8_t  dbg_prompt_shown = 0;
static uint8_t  dbg_prev_active  = 255;   // force first print
static uint32_t dbg_last_brief   = 0;


void setup() {
  Serial.begin(115200);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC_SIGNAL,1000,2000); //us limits
  Serial.println("DEBUG: esc.attach executed");
  esc.writeMicroseconds(1500);
  Serial.println("DEBUG: esc.writeMicroseconds(1500) executed");
 // pinMode(PIN_BUZZ, OUTPUT);
 // digitalWrite(PIN_BUZZ, LOW);

  // ESC output: 50 Hz / 16-bit
  //ledcSetup(ESC_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  //ledcAttachPin(PIN_ESC_SIGNAL, ESC_CH);
  // Arm ESC at neutral
  escArmNeutral(1500, 2000);
  Serial.println("DEBUG: escArmNeutral executed");

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
  Serial.println("DEBUG: angle_setup() executed")

  tuningPromptOnce(); // Print a one-line help and a prompt. Keep it short.

 
}

void loop() {
  /*
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
  */
  static uint32_t t_ctrl = 0;
  static uint32_t t_1s   = 0;

  uint32_t nowMs = millis();

#if DEBUG_ON
  // einmalige Kurz-Hilfe + Prompt (bloß 1x)
  if (!dbg_prompt_shown) {
    Serial.println("Tuning: p+/p- K1, i+/i- K2, s+/s- K4, a+/a- K3, c+/c- calib");
    Serial.print("> ");
    dbg_prompt_shown = 1;
  }
#endif

  // ~loop_time [ms] control loop
  if (nowMs - t_ctrl >= (uint32_t)loop_time) {

#if DEBUG_ON
    // period overrun uyarisi (istege bagli 2 ms tolerans)
    if (nowMs - t_ctrl > (uint32_t)loop_time + 2) {
      Serial.println("WARN: control period overrun");
    }
#endif

    Tuning();
    angle_calc();

    {
      uint8_t active = (vertical && calibrated && !calibrating);

#if DEBUG_ON
      // ACTIVE <-> IDLE kenarinda tek satirlik durum yaz
      if (active != dbg_prev_active) {
        if (active) Serial.println("STATE: ACTIVE (balance loop running)");
        else        Serial.println("STATE: IDLE (Motor_control(0))");
        dbg_prev_active = active;
      }
#endif

      if (active) {
        // Gyro Z in dps (approx like original)
        gyroZ     = GyZ / 131.0f;
        gyroZfilt = alpha * gyroZ + (1.0f - alpha) * gyroZfilt;

        // Reaction wheel speed from analog "encoder"
        motor_speed_enc = getVelocity();

        // State-feedback struktur
        int pwm = (int)(
            175.0f * robot_angle     // K1
          + 16.0f  * gyroZfilt       // K2
          + 13.0f  * motor_speed_enc // K4
          + 0.04f  * motor_speed     // K3
        );

        pwm = constrain(pwm, -255, 255);
        Motor_control(pwm);

#if DEBUG_ON
        // Kisa özet: ~200 ms’de bir tek satir
        if (nowMs - dbg_last_brief >= (uint32_t)DBG_BRIEF_MS) {
          Serial.printf("PWM=%d | ang=%.2f deg, gyro=%.2f dps, w=%.2f\n",
                        pwm, (double)robot_angle, (double)gyroZfilt,
                        (double)motor_speed_enc);
          dbg_last_brief = nowMs;
        }
#endif

        motor_speed += (int32_t)motor_speed_enc;
      } else {
        Motor_control(0);
        motor_speed = 0;
      }
    }

    t_ctrl = nowMs;
  }

  // Once per second: battery & messages
  if (nowMs - t_1s >= 1000) {
    if (!calibrated && !calibrating) {
      Serial.println("Info: Calibrate first (c+ ... c-) to set balance point.");
      Serial.print("> ");
    }

    battVoltageCheck();   // uses calibrated readBatteryVoltage()

    t_1s = nowMs;
  }

}
