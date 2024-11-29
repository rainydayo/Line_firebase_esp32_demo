#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>

// Define WiFi credentials
#define WIFI_SSID "Qwerty4826"
#define WIFI_PASSWORD "oooooooo"

// Define Firebase credentials
#define API_KEY "AIzaSyCvloRko_s2B1XO_fLEsGHDKerDDr3d7R0"
#define DATABASE_URL "https://hopesmartbin-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "john.demohope@gmail.com"
#define USER_PASSWORD "john_demo66"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
unsigned long lastServoRunMillis = 0;
const unsigned long interval12Hours = 12 * 60 * 60 * 1000;

// Define servo motor
Servo myServo;
const int servoPin = 15;
#define LDR_PIN 32

// Thresholds for sensors
const int tempPin = 34; // Example: Analog pin for temperature sensor
const int tdsPin = 35;  // Example: Analog pin for TDS sensor

float tempThresholdLow = 20.0;
float tempThresholdHigh = 50.0;
float tdsThresholdLow = 50.0;
float tdsThresholdHigh = 150.0;

void setup() {
  // Initialize servo motor
  myServo.attach(servoPin);
  //myServo.write(0);
  myServo.write(180);

  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Firebase configuration
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;

  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);
  Firebase.begin(&config, &auth);
  Firebase.setDoubleDigits(5);
  config.timeout.serverResponse = 10 * 1000;

  lastServoRunMillis = millis();
}

void loop() {
  // Firebase-ready check
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    int motorState;
    if (Firebase.RTDB.getInt(&fbdo, "/motor/state", &motorState)) {
      if (motorState == 1) {
        Serial.println("Motor state is 1. Running servo motion.");
        runServoMotion();
        // Reset motor state to 0
        if (!Firebase.RTDB.setInt(&fbdo, "/motor/state", 0)) {
          Serial.println(fbdo.errorReason().c_str());
        }
      }
    } else {
      Serial.println(fbdo.errorReason().c_str());
    }

    // Check and update sensor states
    updateSensorStates();
  }

  if (millis() - lastServoRunMillis >= interval12Hours) {
    Serial.println("12 hours have passed. Running servo motion.");
    int foodState;
    if (Firebase.RTDB.getInt(&fbdo, "/food/state", &foodState)) {
      if (foodState > 50) {
        Serial.println("There is too much food left.");
      } else {
        runServoMotion();
      }
    } else {
      Serial.println(fbdo.errorReason().c_str());
    }
    
    lastServoRunMillis = millis(); // Reset the timer
  }
  
  unsigned int ldrValue = analogRead(LDR_PIN);
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  int ldrState;
  if(ldrValue < 1000){
    ldrState = 1;
  } else {
    ldrState = 0;
  }
  if (!Firebase.RTDB.setInt(&fbdo, "/ldr/state", ldrState)){
    Serial.println(fbdo.errorReason().c_str());
  }
  delay(5000);
}

void runServoMotion() {
  //for (int angle = 0; angle <= 60; angle++) {
  for (int angle = 180; angle >= 120; angle--) {
    myServo.write(angle);
    delay(5); // Delay for smooth motion
  }

  //for (int angle = 60; angle >= 0; angle--) {
  for (int angle = 120; angle <= 180; angle++) {
    myServo.write(angle);
    delay(5); // Delay for smooth motion
  }

  Serial.println("Servo motion completed.");
}

void updateSensorStates() {
  // Read temperature sensor
  float temperature = readTemperature();
  int tempState = evaluateState(temperature, tempThresholdLow, tempThresholdHigh);
  if (!Firebase.RTDB.setInt(&fbdo, "/temp/state", tempState)) {
    Serial.println(fbdo.errorReason().c_str());
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, State: ");
    Serial.println(tempState);
  }

  // Read TDS sensor
  float tdsValue = readTDS();
  int tdsState = evaluateState(tdsValue, tdsThresholdLow, tdsThresholdHigh);
  if (!Firebase.RTDB.setInt(&fbdo, "/quality/state", tdsState)) {
    Serial.println(fbdo.errorReason().c_str());
  } else {
    Serial.print("TDS: ");
    Serial.print(tdsValue);
    Serial.print(" ppm, State: ");
    Serial.println(tdsState);
  }
}

float readTemperature() {
  int analogValue = analogRead(tempPin);
  float voltage = analogValue * (3.3 / 4095.0);
  float resistance = (10.0 * voltage) / (3.3 - voltage); // Assume 10k pull-up resistor
  float tempCelsius = 1.0 / (log(resistance / 10.0) / 3950 + 1.0 / 298.15) - 273.15;
  return tempCelsius;
}

float readTDS() {
  int analogValue = analogRead(tdsPin);
  float voltage = analogValue * (3.3 / 4095.0);
  float tds = (voltage / 3.3) * 1000; // Example calibration
  return tds;
}

int evaluateState(float value, float low, float high) {
  if (value < low) {
    return 0; // Too low
  } else if (value > high) {
    return 2; // Too high
  }
  return 1; // Okay
}
