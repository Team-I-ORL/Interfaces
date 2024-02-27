#ifndef TeensyComm_h
#define TeensyComm_h

#include <Arduino.h>

class TeensyComm {
  public:
    TeensyComm();
    void begin(long baudRate);
    void checkForCommands();
    void setLedPin(int pin);
    bool isMotorActivated(); // Method to check if motor should be activated
    void resetMotorActivation();

  private:
    String inputBuffer;
    bool messageStart;
    int ledPin;
    bool activateMotor;
    void processMessage(String message);
    char calculateChecksum(const String &data);
};

#endif
