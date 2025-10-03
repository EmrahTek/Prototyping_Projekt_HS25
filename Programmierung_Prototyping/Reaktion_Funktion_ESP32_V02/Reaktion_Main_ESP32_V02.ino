/*
Autor: Emrah Tekin
Date:  19.09.2025
Version: 

V0.1 -> Erste version Motor nicht lauft
V0.2 --> Neu Tunning und angle_calc und neu loop

*/

#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>


// Forward declarations from functions_esc.ino are not strictly needed in .ino sketches,
// but we keep prototypes here for clarity when navigating.
void angle_setup(void);
void Motor_control(int pwm);
int  Tuning(void);
void escArmNeutral(uint16_t neutral_us, uint16_t ms_hold );
float getVelocity(void);
void battVoltageCheck(void);
void tuningPromptOnce(void);
int angle_calc_step(void);
void motorTestOnce();

// Globals defined in functions_esc.ino (Arduino concatenates .ino tabs → shared)
extern float alpha, gyroZfilt, robot_angle, loop_time, motor_speed_enc;
extern int16_t GyZ, gyroZ;
extern int32_t motor_speed;
extern bool vertical, calibrating, calibrated;

extern volatile uint8_t g_forceActive;
extern volatile uint8_t g_showIMU; 

extern const int PIN_ESC_SIGNAL, PIN_BUZZ, PIN_VBAT, PIN_SENSOR;
extern const int ESC_CH, ESC_FREQ_HZ, ESC_RES_BITS;

Servo esc;

// ---- DEBUG flags/vars (C-style) ----
#define DEBUG_ON         1
#define DBG_BRIEF_MS     200
static uint8_t  dbg_prev_active  = 255;   // force first print
static uint32_t dbg_last_brief   = 0;


// 1) Hilfsfunktion: Test nur im Setup laufen
static void motorQuickTestSetup() {
  Serial.println("Motor Test (setup): Forward...");
  esc.writeMicroseconds(1800); delay(1000);

  Serial.println("Motor Test (setup): Backward...");
  esc.writeMicroseconds(1200); delay(1000);

  Serial.println("Motor Test (setup): Neutral...");
  esc.writeMicroseconds(1500); delay(1000);

  Serial.println("Motor Test done (setup).");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  /*=========ESC attach (50 Hz)=========*/
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC_SIGNAL,1000,2000); //us limits
  Serial.println("DEBUG: esc.attach executed");
  
  /*====Neutral + arming (with small chunks)====*/
  esc.writeMicroseconds(1500);
  Serial.println("DEBUG: esc.writeMicroseconds(1500) executed");
  escArmNeutral(1500, 2000);
  Serial.println("DEBUG: escArmNeutral executed");

  motorQuickTestSetup();

  // EEPROM for IMU offsets
  EEPROM.begin(64);
  // offsets & 'calibrated' are handled in functions_esc.ino during tuning
  
  // IMU init + gyro bias
  angle_setup();
  Serial.println("DEBUG: angle_setup() executed");

  /* One-line help */
 // tuningPromptOnce(); // Print a one-line help and a prompt. Keep it short.
  

  // Einmaliger Prompt beim Start:
  Serial.println("Tuning: p+/p- (K1), i+/i- (K2), s+/s- (K4), a+/a- (K3), c+/c- (cal), m+/m- (IMU print), f+/f- (force)");
  Serial.write(7);  // PC beep (success)
  esp_task_wdt_config_t cfg = {
    .timeout_ms    = 2000,
    .idle_core_mask= 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&cfg);    // yeni imza
  esp_task_wdt_add(NULL);     // loop task'ı izle

}


void loop() 
{
  static uint32_t t_ctrl = 0;
  static uint32_t t_1s   = 0;
  uint32_t nowMs = millis();

  // ~loop_time [ms] control loop
  if (nowMs - t_ctrl >= (uint32_t)loop_time) {

    #if DEBUG_ON
      // period overrun warning (optional +2 ms tolerance)
      if (nowMs - t_ctrl > (uint32_t)loop_time + 2) {
        Serial.println("WARN: control period overrun");
      }
    #endif

    // ==== Single read of tuning (avoid double call)
    int tuned = Tuning(); 
    if (tuned) {
      Serial.println("Command processed.");
    }
    
    // IMU step (+status bits if needed)
    angle_calc_step();

    // Active decision (forceActive bypass)
    uint8_t active = g_forceActive ? 1 : (vertical && calibrated && !calibrating);  

    #if DEBUG_ON
      if (active != dbg_prev_active) {
        Serial.println(active ? "STATE: ACTIVE (balance loop running)"
                              : "STATE: IDLE (Motor_control(0))");
        dbg_prev_active = active;
      }
    #endif

    if (active) {
      // Gyro Z in dps (approx like original)
      gyroZ     = GyZ / 131.0f;
      gyroZfilt = alpha * gyroZ + (1.0f - alpha) * gyroZfilt;

      // Reaction wheel speed from analog "encoder"
      motor_speed_enc = getVelocity();

      // State-feedback structure
      int pwm = (int)(
          175.0f * robot_angle      // K1
        + 16.0f  * gyroZfilt        // K2
        + 13.0f  * motor_speed_enc  // K4
        + 0.04f  * motor_speed      // K3
      );
      if (pwm > 255)  pwm = 255;
      if (pwm < -255) pwm = -255;

      Motor_control(pwm);

      #if DEBUG_ON
        // short summary: once per ~200 ms
        if (nowMs - dbg_last_brief >= (uint32_t)DBG_BRIEF_MS) {
          Serial.printf("PWM=%d | ang=%.2f deg, gyro=%.2f dps, w=%.2f\n",
                        pwm, (double)robot_angle, (double)gyroZfilt,
                        (double)motor_speed_enc);
          dbg_last_brief = nowMs;
        }
      #endif

      motor_speed += (int32_t)motor_speed_enc;  // crude integral of wheel speed

    } else {
      Motor_control(0);
      motor_speed = 0;
    }
    t_ctrl = nowMs;
    esp_task_wdt_reset();

  }

  // Once per second: battery & messages
  if (nowMs - t_1s >= 1000) {
    if (!calibrated && !calibrating) {
      static uint32_t lastMsg = 0;
      if (nowMs - lastMsg > 5000) { // print help every 5 s to reduce spam
        Serial.println("Info: Calibrate first (c+ ... c-) to set balance point.");
        Serial.print("> ");
        lastMsg = nowMs;
      }
    }

    // battVoltageCheck();   // enable when VBAT wiring is ready
    t_1s = nowMs;
  }
  // loop() içinde, 500 ms’de bir:
    static uint32_t last=0;
  if (millis()-last > 500) {
    Serial.printf("DBG flags: force=%u, vertical=%u, calibrated=%u, calibrating=%u\n",
                (unsigned)g_forceActive, (unsigned)vertical,
                (unsigned)calibrated, (unsigned)calibrating);
    last = millis();
  }
}



