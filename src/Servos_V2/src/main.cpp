#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G3_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 31);
IPAddress receiverComputerIP(192, 168, 1, 35);
const int udpPort = 12345;
WiFiUDP udp;

// Servos
Servo servo_yaw;
servo_pitch;
servo_roll1;
servo_roll2;

// Pines
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// Variables
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
int s1 = 1, s2 = 1;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void receiveOrientationUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';
      JsonDocument doc;
      if (deserializeJson(doc, packetBuffer)) return;
      if (strcmp(doc["device"], "G3_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];
      }
    }
  }
}

float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = fabs(sum - previous);
  previous = sum;
  return diff;
}

void moveServos() {
  float delta = (s1 == 0) ? 40 : 0;
  servo_roll1.write(Gri_roll + delta);
  servo_roll2.write(180 - Gri_roll);
  servo_pitch.write(Gri_pitch);
  servo_yaw.write(Gri_yaw);
}

void sendTorquesUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["torque_yaw"] = Torque_yaw;
  doc["torque_pitch"] = Torque_pitch;
  doc["torque_roll1"] = Torque_roll1;
  doc["torque_roll2"] = Torque_roll2;

  char buffer[256];
  serializeJson(doc, buffer);

  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)buffer, strlen(buffer));
  udp.endPacket();

  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)buffer, strlen(buffer));
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  connectToWiFi();
  udp.begin(udpPort);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

void loop() {
  receiveOrientationUDP();
  moveServos();

  // Leer torques
  Torque_yaw = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);

  sendTorquesUDP();
  delay(10);
}
