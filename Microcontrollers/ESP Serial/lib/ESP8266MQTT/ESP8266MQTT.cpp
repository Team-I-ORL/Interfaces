#include "ESP8266MQTT.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String ESP8266MQTT::lastCommand = "";

ESP8266MQTT::ESP8266MQTT(const char* ssid, const char* password, const char* mqttServer, uint16_t mqttServerPort, const char* mqttUser, const char* mqttPassword, const char* mqttTopicIn, const char* mqttTopicOut)
    : _ssid(ssid), _password(password), _mqttServer(mqttServer), _mqttServerPort(mqttServerPort), _mqttUser(mqttUser), _mqttPassword(mqttPassword), _mqttTopicIn(mqttTopicIn), _mqttTopicOut(mqttTopicOut), _mqttClient(_wifiClient) {}

void ESP8266MQTT::setup() {
    Serial.begin(9600);
    setupWifi();
    _mqttClient.setServer(_mqttServer, _mqttServerPort);
    _mqttClient.setCallback(callback);
    timeClient.begin();
}

void ESP8266MQTT::loop() {
    if (!_mqttClient.connected()) {
        connectToBroker();
    }

    _mqttClient.loop();
    timeClient.update();
}

void ESP8266MQTT::setupWifi() {
    Serial.print("Connecting to ");
    Serial.println(_ssid);
    WiFi.begin(_ssid, _password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("WiFi connected");
}

void ESP8266MQTT::connectToBroker() {
    while (!_mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);

        if (_mqttClient.connect(clientId.c_str(), _mqttUser, _mqttPassword)) {
            Serial.println("connected");
            _mqttClient.subscribe(_mqttTopicIn, 0);
        } else {
            Serial.print("failed, rc=");
            Serial.print(_mqttClient.state());
            Serial.println(" will try again in 5 seconds");
            delay(5000);
        }
    }
}

void ESP8266MQTT::callback(char* topic, byte* payload, unsigned int length) {
    // Clear the last command
    lastCommand = "";

    // Form the command from the payload
    for (unsigned int i = 0; i < length; i++) {
        lastCommand += (char)payload[i];
    }
}

String ESP8266MQTT::getLastCommand() const {
    return lastCommand;
}

void ESP8266MQTT::clearLastCommand() {
    lastCommand = "";
}