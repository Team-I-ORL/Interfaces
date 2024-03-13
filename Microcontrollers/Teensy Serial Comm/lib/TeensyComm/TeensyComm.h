#ifndef TeensyComm_h
#define TeensyComm_h

#include <Arduino.h>

extern volatile bool motorActive;

class TeensyComm {
  public:
    TeensyComm();
    void begin(long baudRate);
    void checkForCommands();
    void setLedPin(int pin);
    bool isMotorActivated(); // Method to check if motor should be activated
    void resetMotorActivation();
    bool isNewCommandAvailable();
    void clearCommand();
    bool newCommandAvailable;


  private:
    String inputBuffer;
    bool messageStart;
    int ledPin;
    bool activateMotor;
    void processMessage(String message);
    char calculateChecksum(const String &data);
};

#endif
