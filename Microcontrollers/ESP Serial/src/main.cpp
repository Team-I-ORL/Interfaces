#include <Arduino.h>
#include "ESP8266MQTT.h"

// Serial communication test
const bool onWifi = false;

// MQTT configuration parameters
const char* ssid = "f31";
const char* password = "isthisok";
const char* mqtt_server = "10.0.0.88"; // e.g., "10.0.0.88"
const uint16_t mqtt_server_port = 1883; 
const char* mqttUser = "mqtt_user";
const char* mqttPassword = "mqtt_password";
const char* mqttTopicIn = "pingpong/ros"; // e.g., "pingpong/ros"
const char* mqttTopicOut = "pingpong/ros"; // e.g., "pingpong/ros"

// Initialize the MQTT client
ESP8266MQTT mqttClient(
    ssid, 
    password, 
    mqtt_server, 
    1883, 
    mqttUser, 
    mqttPassword, 
    mqttTopicIn, 
    mqttTopicOut);

char calculateChecksum(const String &data) {
  char checksum = 0;
  for (unsigned int i = 0; i < data.length(); i++) {
    checksum ^= data[i];
  }
  return checksum;
}
// Function to remove whitespace characters from a string
String removeWhitespace(const String& str) {
    String result;
    for (size_t i = 0; i < str.length(); ++i) {
        if (!isspace(str.charAt(i))) { // Check if the character is not whitespace
            result += str.charAt(i); // Append the character to the result string
        }
    }
    return result;
}
void setup() {
  Serial.begin(9600); // Serial for debugging
  Serial1.begin(9600); // Serial for Teensy communication
  if (onWifi) mqttClient.setup();
}

void loop() {
    String command = "";
  if (onWifi) {
    mqttClient.loop();
    command = mqttClient.getLastCommand();
    command = removeWhitespace(command);
  }
  else {
    command = Serial.readString();
    Serial.println("Serial command: " + command);
  }


  Serial.println("MQTT command: " + command);
  Serial.println("MQTT command length: " + String(command.length()));
  if (!command.isEmpty()) {
      String message = "<" + command + "," + calculateChecksum(command) + ">";
      Serial1.println(message);
      Serial.println("Sent to Teensy: " + message);
      delay(1000);
  }

  mqttClient.clearLastCommand();

  delay(1000); // Adjust as needed
}
