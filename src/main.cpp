#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// DataPacket Structure
typedef struct {
    int x;
    int y;
} DataPacket;

// Current steerAngle Range : 90 +- 55 --> 35 to 145
// Current max speed : 150

// Functional Variables
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
uint8_t steerAngle = 90;
float motorDiff = 1.0;

// Steering Boost Variables
float steerLeftBoost = 1.0;
float steerRightBoost = 1.0;

// Function declaration
void SteeringBoost(uint8_t angle);

DataPacket receivedData;

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received Data - X: ");
    Serial.print(receivedData.x);
    Serial.print(", Y: ");
    Serial.println(receivedData.y);
    Serial.print("Size of Received DataPacket: ");
    Serial.println(len);
}

// *** Wiring connections from ESP32 to TB6612FNG Motor Controller ***
#define AIN1 14 // ESP32 Pin D13 to TB6612FNG Pin AIN1
#define BIN1 27 // ESP32 Pin D12 to TB6612FNG Pin BIN1
#define AIN2 12 // ESP32 Pin D14 to TB6612FNG Pin AIN2
#define BIN2 26 // ESP32 Pin D27 to TB6612FNG Pin BIN2
#define PWMA 13 // ESP32 Pin D26 to TB6612FNG Pin PWMA
#define PWMB 25 // ESP32 Pin D25 to TB6612FNG Pin PWMB

// Create a servo object
Servo steerServo;

// Define the servo pin
int servoPin = 18;

// Steering Boost Function
void SteeringBoost(uint8_t angle)
{
  if (angle > 130)
  {
    steerLeftBoost = 0.6;
  }
  else if (angle < 50)
  {
    steerRightBoost = 0.6;
  }
  else
  {
    steerLeftBoost = 1.0;
    steerRightBoost = 1.0;
  }
}

void setup(void) 
{
  Serial.begin(115200);
  //set all pins as output
  pinMode(AIN2, OUTPUT); 
  pinMode(AIN1, OUTPUT); 
  pinMode(PWMA, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  Serial.println("");

  steerServo.attach(servoPin);
  steerServo.write(90);

  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32_Receiver", "12345678", 1);
  
  // Print SoftAP MAC address
  Serial.print("SoftAP MAC Address: ");
  Serial.println(WiFi.softAPmacAddress());

  if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed on Receiver!");
        return;
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Receiver is ready and listening...");
  
  delay(4000);   

  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(BIN1, LOW);
}

void loop(void) 
{

  // Print the normalized values
  Serial.print("Direction (X): "); Serial.print(receivedData.x);
  Serial.print("  Magnitude (Y): "); Serial.println(receivedData.y);

  // Convert the normalized x-axis value to a servo motor angle
  leftMotorSpeed = rightMotorSpeed = receivedData.y;
  Serial.print("Speed before: "); Serial.println(leftMotorSpeed);

  Serial.print("Speed after: "); Serial.println(static_cast<int>(round(leftMotorSpeed * motorDiff)));

  // Convert the normalized x-axis value to a servo motor angle
  steerAngle = receivedData.x;
  steerServo.write(steerAngle);

  if (leftMotorSpeed < 0) {
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, HIGH);
    leftMotorSpeed = rightMotorSpeed= abs(leftMotorSpeed);
  } else {
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(BIN2, HIGH);
    digitalWrite(BIN1, LOW);
  }

  // Set the motor speed for steer boost
  SteeringBoost(steerAngle);

  // static_cast<int>(round(rightMotorSpeed * motorDiff))

  analogWrite(PWMA, static_cast<int>(round((leftMotorSpeed * steerRightBoost))));   //sets speed, motor A
  analogWrite(PWMB, static_cast<int>(round((rightMotorSpeed * steerLeftBoost))));  //sets speed, motor B

  // Motor and servo debugging function. Use if testing without the glove is needed.
  // debug();

}

void debug(void)
{
    for (int i = 0; i < 250; i++) {
    delay(10);
    analogWrite(PWMA, i);   //sets speed, motor A
    analogWrite(PWMB, i);  //sets speed, motor B
    Serial.print("Speed: "); Serial.println(i);
  }

  for (int i = 250; i > 2; i--) {
    delay(10);
    analogWrite(PWMA, i);   //sets speed, motor A
    analogWrite(PWMB, i);  //sets speed, motor B 
    Serial.print("Speed: "); Serial.println(i);
  }

  for (int i = 90; i >45; i--)
  {
    steerServo.write(i);
    delay(10);
  }
  
  for (int i =90; i < 135; i++) {
    steerServo.write(i);
    delay(10);
  }
}


// ADDITIONAL CODE FOR ESP-NOW COMMUNICATION

// #include <esp_now.h>
// #include <WiFi.h>

// typedef struct {
//     int x;
//     int y;
// } DataPacket;

// DataPacket receivedData;

// void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//     memcpy(&receivedData, incomingData, sizeof(receivedData));
//     Serial.print("Received Data - X: ");
//     Serial.print(receivedData.x);
//     Serial.print(", Y: ");
//     Serial.println(receivedData.y);
//     Serial.print("Size of Received DataPacket: ");
//     Serial.println(len);
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_AP);
//     WiFi.softAP("ESP32_Receiver", "12345678", 1);
//     Serial.print("SoftAP MAC Address: ");
//     Serial.println(WiFi.softAPmacAddress());

//     if (esp_now_init() != ESP_OK) {
//         Serial.println("ESP-NOW Initialization Failed on Receiver!");
//         return;
//     }

//     esp_now_register_recv_cb(onDataRecv);
//     Serial.println("Receiver is ready and listening...");
// }

// void loop() {
// }

// #include <esp_now.h>
// #include <WiFi.h>

// typedef struct {
//     int x;
//     int y;
// } DataPacket;

// DataPacket dataToSend;
// esp_now_peer_info_t peerInfo;
// uint8_t peerAddress[] = {0xEC, 0x62, 0x60, 0x57, 0x23, 0x2D};

// void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     Serial.print("Last Packet Send Status: ");
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
//     Serial.print("Status Code: ");
//     Serial.println(status);
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_STA);

//     if (esp_now_init() != ESP_OK) {
//         Serial.println("ESP-NOW Initialization Failed on Transmitter!");
//         return;
//     }

//     esp_now_register_send_cb(onDataSent);
//     memcpy(peerInfo.peer_addr, peerAddress, sizeof(peerAddress));
//     peerInfo.channel = 1;
//     peerInfo.encrypt = false;

//     if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//         Serial.println("Failed to add peer!");
//     } else {
//         Serial.println("Peer added successfully!");
//     }
// }

// void loop() {
//     dataToSend.x = random(-100, 100);
//     dataToSend.y = random(0, 100);

//     Serial.print("Sending to MAC Address: ");
//     for (int i = 0; i < 6; i++) {
//         Serial.printf("%02X", peerInfo.peer_addr[i]);
//         if (i < 5) Serial.print(":");
//     }
//     Serial.println();

//     esp_now_send(peerAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

//     Serial.print("Sent Data - X: ");
//     Serial.print(dataToSend.x);
//     Serial.print(", Y: ");
//     Serial.println(dataToSend.y);

//     Serial.print("Size of DataPacket: ");
//     Serial.println(sizeof(dataToSend));

//     delay(1000);
// }