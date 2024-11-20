#include <WiFi.h>
#include <Firebase_ESP_Client.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "AIS_Nutcha"
#define WIFI_PASSWORD "moodaeng05"

/* 2. Define the API Key */
#define API_KEY "AIzaSyCvloRko_s2B1XO_fLEsGHDKerDDr3d7R0"

/* 3. Define the RTDB URL */
#define DATABASE_URL "https://hopesmartbin-default-rtdb.asia-southeast1.firebasedatabase.app/" 

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "john.demohope@gmail.com"
#define USER_PASSWORD "john_demo66"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

const int ledPin = 4;
const int tempPin = 34; // Analog pin for B3950
const int tdsPin = 35;  // Analog pin for TDS sensor

float tempThresholdLow = 20.0;  // Example: Minimum temperature for "okay"
float tempThresholdHigh = 30.0; // Example: Maximum temperature for "okay"
float tdsThresholdLow = 300;    // Example: Minimum TDS for "okay"
float tdsThresholdHigh = 700;   // Example: Maximum TDS for "okay"

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200);
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

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);
  Firebase.begin(&config, &auth);
  Firebase.setDoubleDigits(5);
  config.timeout.serverResponse = 10 * 1000;
}

void loop() {
  // Firebase.ready() should be called repeatedly to handle authentication tasks.
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Update LED state
    int ledState;
    if (Firebase.RTDB.getInt(&fbdo, "/led/state", &ledState)) {
      digitalWrite(ledPin, ledState);
    } else {
      Serial.println(fbdo.errorReason().c_str());
    }

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
}

float readTemperature() {
  // Simulate reading the temperature sensor
  int analogValue = analogRead(tempPin);
  float voltage = analogValue * (3.3 / 4095.0); // Convert to voltage
  float resistance = (10.0 * voltage) / (3.3 - voltage); // Assume 10k pull-up
  float tempCelsius = 1.0 / (log(resistance / 10.0) / 3950 + 1.0 / 298.15) - 273.15;
  return tempCelsius;
}

float readTDS() {
  // Simulate reading the TDS sensor
  int analogValue = analogRead(tdsPin);
  float voltage = analogValue * (3.3 / 4095.0); // Convert to voltage
  float tds = (voltage / 3.3) * 1000; // Scale TDS based on sensor calibration
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
