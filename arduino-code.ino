#include <DHT.h>
#include <FastLED.h>
// Define the DHT pins and sensor types
#define DHTPIN_S1 11
#define DHTPIN_S2 10
#define DHTPIN_S4 13
#define DHTPIN_S5 5
#define DHTTYPE DHT22
#define DATA_PIN 6   // Change this to the pin connected to the LED strip
CRGB leds[17];
// Define the relay pins
#define RELAY_PIN 9        // Compressor relay
#define DEFROST_HEATER_PIN 7    // Heater relay for defrosting
#define HUMIDITY_HEATER_PIN 8 // Heater relay for humidity
#define FAN_PIN 4
#define LED_PIN 3 

// Define the ice sensor pin
#define ICE_SENSOR_PIN 22

// Define temperature thresholds
#define MAX_TEMP 8
#define MIN_TEMP 6
#define DEFROST_TEMP_DROP 2.0
#define HEAT_ON_TEMP_OFFSET -2.0 // Temperature offset for turning on the heater
#define HUMIDITY_TEMP_DROP 2.0  // Temperature drop for humidity control

// Define humidity thresholds
#define MAX_HUMIDITY 60
#define MIN_HUMIDITY 50
#define HUMIDITY_TOLERANCE  10// Allowable difference between S4 and target humidity
// Define timing variables
unsigned long previousMillis = 0;
const long interval = 10000;
unsigned long lastFridgeOffTime = 0;
const long compressorDelay = 180000;
bool firstRun = true;

// Defrosting variables
bool isDefrosting = false;
unsigned long defrostStartTime = 0;
const long defrostDuration = 300000;
const long defrostInterval = 10800000;
unsigned long lastDefrostTime = 0;
float currentMaxTemp;
bool defrostActive = false;

// Humidity control variables
bool checkingHumidity = false;
float targetHumidity;
float tempBeforeHumidityCheck;
unsigned long humidityCheckStartTime;
const long humidityCheckDuration = 60000; // 1 minute

// Create DHT objects for each sensor
DHT dhtS1(DHTPIN_S1, DHTTYPE);
DHT dhtS2(DHTPIN_S2, DHTTYPE);
DHT dhtS4(DHTPIN_S4, DHT11);
DHT dhtS5(DHTPIN_S5, DHTTYPE);

// Function to control the compressor relay
void rainbow() {
  // Fill the LED strip with a rainbow pattern
  for (int i = 0; i < 17; i++) {
    leds[i] = CHSV((i * 256 / 17) + (millis() / 10), 255, 255); // Create a rainbow effect
  }
  FastLED.show(); // Update the LED strip
}

void controlCompressor(int state) {
  if (state == LOW) { // Turn compressor ON
    if (firstRun || (millis() - lastFridgeOffTime >= compressorDelay)) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println(F("Compressor ON"));
      if (firstRun) {
        firstRun = false;
      }
    } else {
      Serial.println(F("Waiting to turn compressor ON..."));
      Serial.print(F("Time remaining: "));
      Serial.print((compressorDelay - (millis() - lastFridgeOffTime)) / 1000);
      Serial.println(F(" seconds"));
    }
  } else { // Turn compressor OFF
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println(F("Compressor OFF"));
    lastFridgeOffTime = millis();
  }
}

// Function to control the defrosting heater relay
void controlDefrostHeater(int state) {
  digitalWrite(DEFROST_HEATER_PIN, state);
  if (state == LOW) {
    Serial.println(F("Defrost Heater ON"));
  } else {
    Serial.println(F("Defrost Heater OFF"));
  }
}

// Function to control the humidity heater relay
void controlHumidityHeater(int state) {
  digitalWrite(HUMIDITY_HEATER_PIN, state);
  if (state == LOW) {
    Serial.println(F("Humidity Heater ON"));
  } else {
    Serial.println(F("Humidity Heater OFF"));
  }
}

// Function to control the fan relay - Not used in the current logic, but kept for potential future use
void controlFan(int state) {
  digitalWrite(FAN_PIN, state);
  if (state == LOW) {
    Serial.println(F("Fan ON"));
  } else {
    Serial.println(F("Fan OFF"));
  }
}

void setup() {
  Serial.begin(9600);
  delay(5000);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, 17);
  // Light up the LEDs in rainbow pattern
  rainbow();
  // Initialize DHT sensors
  dhtS1.begin();
  dhtS2.begin();
  dhtS4.begin();
  dhtS5.begin();
  delay(2000);
  // Initialize relay pins
  pinMode(RELAY_PIN, OUTPUT);        // Compressor
  pinMode(DEFROST_HEATER_PIN, OUTPUT); // Defrost Heater
  pinMode(HUMIDITY_HEATER_PIN, OUTPUT); // Humidity Heater
  pinMode(FAN_PIN, OUTPUT);            // Fan
  pinMode(LED_PIN, OUTPUT);
  // Initialize ice sensor pin
  pinMode(ICE_SENSOR_PIN, INPUT);
  digitalWrite(ICE_SENSOR_PIN, LOW);
  // Initialize relays to OFF state
  digitalWrite(RELAY_PIN, HIGH);        // Compressor
  digitalWrite(DEFROST_HEATER_PIN, HIGH); // Defrost Heater
  digitalWrite(HUMIDITY_HEATER_PIN, HIGH); // Humidity Heater
  digitalWrite(FAN_PIN, LOW);            // Fan
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  currentMaxTemp = MAX_TEMP;
}

void loop() {
  unsigned long currentMillis = millis();
  // Check if it's time to check the temperature
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Read data from DHT sensors
    float humidityS1 = dhtS1.readHumidity();
    float temperatureS1C = dhtS1.readTemperature();
    float humidityS2 = dhtS2.readHumidity();
    float temperatureS2C = dhtS2.readTemperature();
    float humidityS4 = dhtS4.readHumidity();
    float temperatureS4C = dhtS4.readTemperature();
    float humidityS5 = dhtS5.readHumidity();
    float temperatureS5C = dhtS5.readTemperature();
    // Check if any reads failed
    int validReadings = 0;
    float totalTemperature = 0.0;
    float totalHumidity = 0.0;
    if (!isnan(humidityS1) && !isnan(temperatureS1C)) {
      totalTemperature += temperatureS1C;
      totalHumidity += humidityS1;
      validReadings++;
    }
    if (!isnan(humidityS2) && !isnan(temperatureS2C)) {
      totalTemperature += temperatureS2C;
      totalHumidity += humidityS2;
      validReadings++;
    }
    if (validReadings < 1) {
      Serial.println(F("Too many sensor failures! Check sensors."));
      delay(1000);
      return;
    }
    // Calculate the average temperature (excluding S4 and S5 for initial temp control)
    float averageTemperature = totalTemperature / validReadings;
    float averageHumidity = totalHumidity / validReadings;
    // Print the sensor data to the Serial Monitor
    Serial.print(F("Sensor S1 - Temperature: "));
    if (isnan(temperatureS1C))
      Serial.print(F("Failed "));
    else
      Serial.print(temperatureS1C);
    Serial.print(F(" *C  Humidity: "));
    if (isnan(humidityS1))
      Serial.print(F("Failed "));
    else
      Serial.print(humidityS1);
    Serial.print(F(" %  "));
    Serial.print(F("Sensor S2 - Temperature: "));
    if (isnan(temperatureS2C))
      Serial.print(F("Failed "));
    else
      Serial.print(temperatureS2C);
    Serial.print(F(" *C  Humidity: "));
    if (isnan(humidityS2))
      Serial.print(F("Failed "));
    else
      Serial.print(humidityS2);
    Serial.print(F(" %  "));
    Serial.print(F("Sensor S4 - Temperature: "));
    if (isnan(temperatureS4C))
      Serial.print(F("Failed "));
    else
      Serial.print(temperatureS4C);
    Serial.print(F(" *C  Humidity: "));
    if (isnan(humidityS4))
      Serial.print(F("Failed "));
    else
      Serial.print(humidityS4);
    Serial.print(F(" %  "));
    Serial.print(F("Sensor S5 - Temperature: "));
    if (isnan(temperatureS5C))
      Serial.print(F("Failed "));
    else
      Serial.print(temperatureS5C);
    Serial.print(F(" *C  Humidity: "));
    if (isnan(humidityS5))
      Serial.print(F("Failed "));
    else
      Serial.print(humidityS5);
    Serial.print(F(" %  "));
    Serial.print(F("Average Temperature: "));
    Serial.print(averageTemperature);
    Serial.print(F(" *C  "));
    Serial.print(F("Average Humidity: "));
    Serial.print(averageHumidity);
    Serial.println(F(" %"));

    // Check defrost interval
    if (currentMillis - lastDefrostTime >= defrostInterval) {
      lastDefrostTime = currentMillis;
      isDefrosting = true;
      defrostStartTime = currentMillis;
      Serial.println(F("Starting defrost cycle."));
      currentMaxTemp = MAX_TEMP - DEFROST_TEMP_DROP;
      defrostActive = true;
    }

    // Defrosting process
    if (isDefrosting) {
      if (digitalRead(ICE_SENSOR_PIN) == HIGH) {
        controlCompressor(HIGH);          // Turn off compressor
        controlDefrostHeater(LOW);        // Turn on defrost heater
        controlHumidityHeater(HIGH);
        controlFan(HIGH);            // Ensure fan is off
        Serial.println(F("Ice detected, defrosting..."));
      }
      if (currentMillis - defrostStartTime >= defrostDuration) {
        isDefrosting = false;
        controlDefrostHeater(HIGH);       // Turn off defrost heater
        Serial.println(F("Defrost cycle complete."));
        currentMaxTemp = MAX_TEMP;
        defrostActive = false;
      }
    }

    // Temperature and Humidity Control Logic
    if (!checkingHumidity) { // Normal temperature control
      if (averageTemperature > currentMaxTemp) {
        controlCompressor(LOW); // Turn the compressor ON
        controlDefrostHeater(HIGH);     // Ensure defrost heater is OFF
        controlHumidityHeater(HIGH);
      } else if (averageTemperature < (MIN_TEMP + HEAT_ON_TEMP_OFFSET)) {
        controlCompressor(HIGH); // Turn the compressor OFF
        controlDefrostHeater(HIGH);     // Ensure defrost heater is OFF
        controlHumidityHeater(HIGH);
      } else if (averageTemperature < MIN_TEMP) {
        controlCompressor(HIGH);
        controlDefrostHeater(HIGH);
        controlHumidityHeater(HIGH);
      }
      else if (averageTemperature <= MAX_TEMP && averageTemperature >= MIN_TEMP) {
        // Temperature is within range, check humidity
        if (averageHumidity < MIN_HUMIDITY || averageHumidity > MAX_HUMIDITY) {
          // Humidity is out of range, lower temperature
          checkingHumidity = true;
          targetHumidity = averageHumidity;  //set target
          tempBeforeHumidityCheck = averageTemperature;
          humidityCheckStartTime = currentMillis;
          controlCompressor(LOW);            // Turn on compressor for humidity check
          controlDefrostHeater(HIGH);
          controlHumidityHeater(LOW);
          Serial.println(F("Humidity out of range.  Compressor ON, Humidity heater ON for humidity control."));
        }
        else
        {
           controlCompressor(HIGH);
           controlDefrostHeater(HIGH);
           controlHumidityHeater(HIGH);
           Serial.println(F("Temperature and Humidity within range."));
        }
      }
      else {
        controlCompressor(HIGH); //basic operation, cooling is off.
        controlDefrostHeater(HIGH);
        controlHumidityHeater(HIGH);
        Serial.println(F("Fridge maintaining temperature"));
      }
    } else { // Humidity check in progress
      if (averageTemperature > tempBeforeHumidityCheck - HUMIDITY_TEMP_DROP) {
        controlCompressor(LOW);          // Compressor ON
        controlDefrostHeater(HIGH);          // Ensure defrost heater is OFF
        controlHumidityHeater(LOW);           // Turn ON humidity heater
        Serial.println(F("Temperature decreased,  Compressor ON, Humidity heater ON for humidity control"));
      }
       if (abs(humidityS4 - targetHumidity) <= HUMIDITY_TOLERANCE) {
        Serial.println(F("Humidity level reached. Keeping Compressor ON and humidity heater ON for 1 minute."));
      }
      if (currentMillis - humidityCheckStartTime >= humidityCheckDuration) {
        controlHumidityHeater(HIGH);         // Turn off humidity heater after 1 minute
        checkingHumidity = false;           // Reset humidity check flag
        Serial.println(F("Humidity check complete.  Returning to normal operation."));
        currentMaxTemp = MAX_TEMP;
      }
    }
  }
}