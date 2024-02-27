#ifndef ESP8266MQTT_h
#define ESP8266MQTT_h

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class ESP8266MQTT {
public:
    ESP8266MQTT(const char* ssid, const char* password, const char* mqttServer, uint16_t mqttServerPort, const char* mqttUser, const char* mqttPassword, const char* mqttTopicIn, const char* mqttTopicOut);
    void setup();
    void loop();
    String getLastCommand() const; 
    void clearLastCommand();

private:
    const char* _ssid;
    const char* _password;
    const char* _mqttServer;
    uint16_t _mqttServerPort;
    const char* _mqttUser;
    const char* _mqttPassword;
    const char* _mqttTopicIn;
    const char* _mqttTopicOut;

    WiFiClient _wifiClient;
    PubSubClient _mqttClient;

    void setupWifi();
    void connectToBroker();

    static String lastCommand; // Make lastCommand static

    static void callback(char* topic, byte* payload, unsigned int length);
};

#endif
