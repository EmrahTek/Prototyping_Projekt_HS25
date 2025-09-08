





#include <DFRobot_DHT20.h>



// FireBeetle ESP32 V4.0 LED + Hello World

# define LED_PIN 2 // GPIO2 



void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);

  Serial.println("ESP32 FireBeetle V4.0 wurde begonnen!");

}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Hello World - LED ON");
  delay(1000);

  // LED sön
  digitalWrite(LED_PIN, LOW);
  Serial.println("Hello World - LED OFF");
  delay(1000);

}
