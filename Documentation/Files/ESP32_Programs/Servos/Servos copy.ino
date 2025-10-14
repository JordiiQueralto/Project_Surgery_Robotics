#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h> //needed for I2C to read IMU
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G5_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 51); // IP address of the receiver ESP32 G5_Gripper
IPAddress receiverComputerIP(192, 168, 1, 55); // IP address of your G5-computer
const int udpPort = 12345;
WiFiUDP udp;

// Servo settings
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1; 
Servo servo_roll2; // DECLARACIÓ DE SERVOS

// PINS DELS SERVOS
// Yaw
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
// Pitch
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
// Roll 1
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
// Roll 2
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

// Valors de les resistències shunt (son totes iguals)
const float Rshunt = 1.6; 

// Variables to store received data
// Variables per guardar la posició del gripper
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
// Variables per guardar els torques de cada servo
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
// Variables per a guardar les posició dels servos 
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
// Variables per a guardar la posició dels servos
float OldValueRoll = 0, OldValuePitch = 0, OldValueYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
// Variables per a guardar l'estat dels botons
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
      Serial.print("Received packet size: ");
      Serial.println(packetSize);
      Serial.print("Received: ");
      Serial.println((char*)packetBuffer);

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G5_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];
        Serial.print("Gri_Roll: "); Serial.print(Gri_roll); Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch); Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw); Serial.print("S1: "); Serial.print(s1); Serial.print(" S2: "); Serial.println(s2);
      } else {
        Serial.println("Unknown device.");
      }
    }
  }
}

// Functions for Servos

// Funció per calcular el corrent a partir del valor llegit del ADC
// El corrent es calculat a partir de la lectura del ADC i la resistència shunt fent servir la llei d'Ohm I = V/R
// on V = (ADC_value / 4095) * 3.3V (ja que el ADC és de 12 bits i la tensió màxima és de 3.3V)

float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt; // Converteix el valor ADC en voltatge i després en corrent
  }
  return integratedCurrent;
}

// Funció per calcular el torque a partir del corrent

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin); // Llegeix el corrent del pin analògic
  sum += current; // Suma el corrent al valor acumulat
  float diff = abs(sum - previous); // Calcula la diferència entre el valor actual i el valor anterior
  previous = sum; // Actualitza el valor anterior
  return diff; // Retorna la diferència com a torque
}

// Funció per a moure els servos a la posició del gripper

void moveServos() {
  roll = Gri_roll;
  OldValueRoll = roll;
  pitch = Gri_pitch;
  OldValuePitch = pitch;
  yaw = Gri_yaw;
  OldValueYaw = yaw;

  // Control de servos 
  float delta = 0;  // valor base
  if (s1 == 0) {
    delta = 40;  // obrir (se separen)
    Serial.println("S1 premut → Obrint");
  }
  // Escriu les posicions corregides
  servo_roll1.write(Gri_roll + delta);
  servo_roll2.write(180 - Gri_roll);
  servo_pitch.write(pitch);
  servo_yaw.write(yaw);

}


// Funció per a inicialitzar els servos i el MPU-9250
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

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

  // Començar amb els servos en la posició 90 (neutral)
  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

void loop() {
  receiveOrientationUDP();
  moveServos(); // Move servos to the position of the gripper
  delay(10);
}