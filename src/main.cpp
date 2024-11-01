#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define CHANNEL 1

uint8_t newData;

// Create a servo object
Servo steerServo;

// Define the servo pin
int servoPin = 18;

/** callback when data is recv from Master **/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.print("I just received -> ");
  Serial.println(*data);
  memcpy(&newData, data, sizeof(newData));
}

void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP("RX_1", "RX_1_Password", CHANNEL, 0);
  
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  steerServo.attach(servoPin);
}

void loop() {
  Serial.print("I did this to data -> ");
  Serial.println(newData);

  // Write the angle to the servo motor
  steerServo.write(newData);
}

