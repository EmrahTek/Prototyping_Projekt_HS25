
/*

This program will connect to the FireBeetle Board via I2C using the HW-123 MPU6050 gyroscope and control the RHS500 motor. 

Autor: Emrah Tekin
Version: V1.0 
Date: 09.09.2025

*/




#include <DFRobot_DHT20.h>
#include <Wire.h>




// FireBeetle ESP32 V4.0 LED + Hello World
# define SDA_PIN 21
# define SCL_PIN 22
# define LED_PIN 2 // GPIO2 


// AD0=GND -> 0x68, AD0=VCC -> 0x69
uint8_t MPU_ADDR = 0x68;

// MPU6050 register addresses
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Sensitivity scale factors (from datasheet)
const float ACCEL_SENS_2G = 16384.0f; // LSB/g
const float GYRO_SENS_250 = 131.0f;   // LSB/(deg/s)
void i2cWrite8(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom((int)dev, (int)len, (int)true);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}
int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

void i2cScan() {
  Serial.println(F("I2C address scan:"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F(" - Device found at 0x"));
      Serial.println(addr, HEX);
    }
  }
}

bool mpuInit() {
  // Wake up device (clear sleep bit)
  i2cWrite8(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  delay(100);

  // DLPF = 3 (≈44 Hz accel / 42 Hz gyro), reduces noise
  i2cWrite8(MPU_ADDR, REG_CONFIG, 0x03);

  // Gyro range: ±250 dps
  i2cWrite8(MPU_ADDR, REG_GYRO_CONFIG, 0x00);

  // Accel range: ±2g
  i2cWrite8(MPU_ADDR, REG_ACCEL_CONFIG, 0x00);

  // Sample rate: 1 kHz / (1 + 9) = 100 Hz
  i2cWrite8(MPU_ADDR, REG_SMPLRT_DIV, 9);

  // Simple check: first read attempt
  uint8_t test[2] = {0};
  i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, test, 2);
  return Wire.available() == 0;
}




void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);

  
  // Initialize I2C bus at 400 kHz
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  delay(50);

  i2cScan();

  // If 0x68 not found, try 0x69
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    MPU_ADDR = 0x69;
    Serial.println(F("0x68 not found, trying 0x69..."));
  }

  if (!mpuInit()) {
    Serial.print(F("MPU initialized, address: 0x"));
    Serial.println(MPU_ADDR, HEX);
  } else {
    Serial.println(F("MPU init may have failed (check wiring/address)."));
  } 

}  





void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Hello World - LED ON");
  delay(1000);

  // LED sön
  digitalWrite(LED_PIN, LOW);
  Serial.println("Hello World - LED OFF");
  delay(1000);



  
  uint8_t raw[14];
  i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, raw, 14);

  int16_t ax_raw = toInt16(raw[0], raw[1]);
  int16_t ay_raw = toInt16(raw[2], raw[3]);
  int16_t az_raw = toInt16(raw[4], raw[5]);
  // int16_t temp_raw = toInt16(raw[6], raw[7]); // optional temperature
  int16_t gx_raw = toInt16(raw[8], raw[9]);
  int16_t gy_raw = toInt16(raw[10], raw[11]);
  int16_t gz_raw = toInt16(raw[12], raw[13]);

  // Convert to physical units
  float ax = ax_raw / ACCEL_SENS_2G; // g
  float ay = ay_raw / ACCEL_SENS_2G; // g
  float az = az_raw / ACCEL_SENS_2G; // g

  float gx = gx_raw / GYRO_SENS_250; // deg/s
  float gy = gy_raw / GYRO_SENS_250; // deg/s
  float gz = gz_raw / GYRO_SENS_250; // deg/s

  // Print results
  Serial.print(F("ACC [g] ax: ")); Serial.print(ax, 3);
  Serial.print(F("  ay: "));       Serial.print(ay, 3);
  Serial.print(F("  az: "));       Serial.print(az, 3);
  Serial.print(F(" | GYRO [dps] gx: ")); Serial.print(gx, 2);
  Serial.print(F("  gy: "));            Serial.print(gy, 2);
  Serial.print(F("  gz: "));            Serial.println(gz, 2);

  delay(10); // ~100 Hz loop
  

}

/*

Connection reminder:

VCC → 3.3 V

GND → GND

SDA → GPIO21

SCL → GPIO22

AD0 → GND (0x68) or 3.3 V (0x69)




*/