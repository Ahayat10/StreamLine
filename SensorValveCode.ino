#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h> // Include the Ticker library for ESP32 hardware timer
#include <SPIFFS.h>
#include <DHT.h>

#define ss 5
#define rst 14
#define dio0 2
#define HMC5883L_ADDR 0x1E // I2C 7bit address of HMC5883
#define FLOWSENSORPIN 4 // Flow sensor pin
#define DHT_SENSOR_PIN  33 // ESP32 pin GPIO32 connected to DHT11 sensor
#define DHT_SENSOR_TYPE DHT11

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
const int sensor_pin = 27; // Soil moisture sensor O/P pin
const long loraInterval = 2000; 
unsigned long previousLoraMillis = 0; // will store last time LoRa data was sent
int flag = 0;
float moisture_percentage;
const int sigPin = 32; 

bool haveHMC5883L = false;
const int relay_pin = 12; // Define the relay pin 

float humi, tempF;

// Variables for flow sensor
volatile uint16_t pulses = 0;
volatile uint8_t lastflowpinstate;
Ticker flowMeterTicker; // Ticker for flow meter

void readFlowSensor() {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  if (x != lastflowpinstate) {
    if (x == HIGH) {
      pulses++;
    }
    lastflowpinstate = x;
  }
}


bool detectHMC5883L() {
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(10); // Select Identification register A
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_ADDR, 3);
    if (3 == Wire.available()) {
        char a = Wire.read();
        char b = Wire.read();
        char c = Wire.read();
        if (a == 'H' && b == '4' && c == '3')
            return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("LoRa Sender");
    
    // initialize the tilt switch pin as an input:
    pinMode(sigPin, INPUT);  

    LoRa.setPins(ss, rst, dio0);
    while (!LoRa.begin(915E6)) {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initializing OK!");

    Wire.begin(21, 22); // Initialize I2C communication for HMC5883L
    pinMode(relay_pin, OUTPUT); // Initialize the relay pin as an output
    
    pinMode(FLOWSENSORPIN, INPUT_PULLUP); // Initialize flow sensor pin
    lastflowpinstate = digitalRead(FLOWSENSORPIN);
    flowMeterTicker.attach_ms(1, readFlowSensor); // Call readFlowSensor every 1 millisecond
    
    if (!SPIFFS.begin(true)) {
        Serial.println("Formatting SPIFFS. This may take a while...");
        SPIFFS.format();
        if (!SPIFFS.begin(true)) {
            Serial.println("Failed to format SPIFFS");
            return;
        } else {
            Serial.println("SPIFFS formatted successfully");
        }
    } else {
        Serial.println("SPIFFS already mounted");
    }
}

void loop() {
  String LoRaData;
  // Soil moisture sensor reading
  unsigned long currentMillis = millis();
  humi = dht_sensor.readHumidity();
  tempF = dht_sensor.readTemperature(true);

  int sensor_analog = analogRead(sensor_pin);
  moisture_percentage = map(sensor_analog, 1086, 4095, 100, 0);

  // HMC5883L magnetometer reading
  int16_t x = 0, y = 0, z = 0;
  if (detectHMC5883L()) {
    if (!haveHMC5883L) {
        haveHMC5883L = true;
        Wire.beginTransmission(HMC5883L_ADDR);
        Wire.write(0x02); // Select mode register
        Wire.write(0x00); // Continuous measurement mode
        Wire.endTransmission();
    }

    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x03); // Select register 3, X MSB register
    Wire.endTransmission();

    Wire.requestFrom(HMC5883L_ADDR, 6);
    if (6 <= Wire.available()) {
        x = (int16_t)(Wire.read() << 8 | Wire.read()); // X msb and lsb
        z = (int16_t)(Wire.read() << 8 | Wire.read()); // Z msb and lsb
        y = (int16_t)(Wire.read() << 8 | Wire.read()); // Y msb and lsb
    }
  } else if (haveHMC5883L) {
      haveHMC5883L = false;
      Serial.println("Lost connection to HMC5883L!");
  }

  // Flow sensor reading
  float liters = pulses;
  liters /= 7.5; // Conversion factor for plastic sensor
  liters /= 60.0;

  dht_sensor.begin(); 

  // Continuously check for incoming messages
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
      // Clear previous data
      LoRaData = "";

      // Read packet
      while (LoRa.available()) {
          char c = LoRa.read();
          if (c >= 32 && c <= 126) {
              LoRaData += c;
          }
      }

      // Check for "ON" or "OFF" command in LoRaData
      if (LoRaData.indexOf("ON") != -1) {
          flag = 1; // Set flag for relay ON
      } else if (LoRaData.indexOf("OFF") != -1) {
          flag = 0; // Set flag for relay OFF
      }

      Serial.print("Received packet: ");
      Serial.print(LoRaData);
      Serial.print(" with RSSI ");
      Serial.println(LoRa.packetRssi());
  }
  // Check if it's time to send LoRa data
  if (currentMillis - previousLoraMillis >= loraInterval) {
      previousLoraMillis = currentMillis;

      // Construct and send the LoRa message
      String flowString = String(liters, 1) + " Liters";
      String humiString = String(humi);
      String tempString = String(tempF);
      String message = "Humidity: " + humiString + ", Temp: " + tempString + " , Moisture Level: " + String(moisture_percentage, 1) + "%, " +
                      "x: " + String(x) + " y: " + String(y) + " z: " + String(z) + ", " +
                      "Flow: " + flowString;

      Serial.println(message);
      LoRa.beginPacket();
      LoRa.print(message);
      LoRa.endPacket();
  }

  // Control the relay based on moisture level or magnetometer reading
  // Only if no "ON" or "OFF" command was received
  if (moisture_percentage > 50 || x > 300 || x < -100) {
      flag = 0; // Conditions to turn the relay OFF
  } 
  else if (flag != 1) { // Only turn off the relay if it's not already turned on by LoRa command
    flag = 0; // If moisture is above 60%, turn the relay OFF
  }
  else{
    flag = 1;
  }

  digitalWrite(relay_pin, flag == 1 ? HIGH : LOW);

}