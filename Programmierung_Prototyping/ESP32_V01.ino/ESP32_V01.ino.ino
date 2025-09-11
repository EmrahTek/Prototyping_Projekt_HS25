
/*

This program will connect to the FireBeetle Board via I2C using the HW-123 MPU6050 gyroscope and control the RHS500 motor. 

Autor: Emrah Tekin
Version: V1.0 
Date: 09.09.2025

*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// --- Adjust these for your wiring ----
#ifndef SDA_PIN
#define SDA_PIN 21 // FireBeetle ESP32 V4: generel GPIO21
#endif
#ifndef SCL_PIN
#define SCL_PIN 22 // FireBeetle ESP32 Vç: generel GPIO22
#endif

// If AD0=GND -> 0x68, if AD0=3V3 -> 0x69
//uint8_t MPU_ADDR = 0x68;   

// FireBeetle ESP32 V4.0 LED + Hello World
# define LED_PIN 2 // GPIO2 

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  pinMode(LED_PIN, OUTPUT);



/*
  //Start I2C with explicit pins & 400kHz
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  Serial.println("\nMPU6050 init...");

  // Try to initalize (Provide address & bus & i2c clock)
  if (!mpu.begin(MPU_ADDR, &Wire, 400000)){
    Serial.println("Failed to find MPU6050 chip at 0x68. Trying 0x69...");
    // Fallback
    MPU_ADDR = 0x69;
    if(!mpu.begin(MPU_ADDR, &Wire, 400000)){
      Serial.println("MPU6050 nor found at 0x69. Check wiring & power. ");
      // I2C scanner tipp:
      Serial.println("Tipp: Run an I2C scanner to verify address.");
      while(1){delay(1000);}
    }
  } */

 // Serial.print("MPU6050 Found at 0x");
  // Serial.println(MPU_ADDR, HEX);

 // mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //Acceleration range = sets how much g it can measure.
 //  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Gyroscope range = sets how fast it can measure turns.
 //  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Filter bandwidth = adjusts the noise/response time balance.

  // Print effective settings
  //************************************************************//
  /*This code only exists to show the user what the setting is.
    It checks the sensor and prints for debug purposes to see 
    if the correct range is selected.*/
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()){
    case MPU6050_RANGE_2_G: Serial.println("+/- 2G"); break; 
    case MPU6050_RANGE_4_G:  Serial.println("+/- 4G"); break;
    case MPU6050_RANGE_8_G:  Serial.println("+/- 8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("+/-16G"); break;
  }
  //*********************************************************//
  /*This code only writes information to the serial port for debug → 
  "The measurement range of the gyroscope is currently set to ±XXX degrees/second." */
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:  Serial.println("+/- 250 deg/s"); break;
    case MPU6050_RANGE_500_DEG:  Serial.println("+/- 500 deg/s"); break;
    case MPU6050_RANGE_1000_DEG: Serial.println("+/-1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("+/-2000 deg/s"); break;
  }
  //*******************************************************//
  /* This code only prints the value (in Hz) 
  of the DLPF filter selected in the MPU6050 sensor to the serial port.*/
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ:  Serial.println("94 Hz");  break;
    case MPU6050_BAND_44_HZ:  Serial.println("44 Hz");  break;
    case MPU6050_BAND_21_HZ:  Serial.println("21 Hz");  break;
    case MPU6050_BAND_10_HZ:  Serial.println("10 Hz");  break;
    case MPU6050_BAND_5_HZ:   Serial.println("5 Hz");   break;
  }
  //***********************************************************//
  Serial.println("Setup OK.");
  delay(100); 

}  

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Hello World - LED ON");
  delay(1000);

  // LED sön
  digitalWrite(LED_PIN, LOW);
  Serial.println("Hello World - LED OFF");
  delay(1000);


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
  

}

