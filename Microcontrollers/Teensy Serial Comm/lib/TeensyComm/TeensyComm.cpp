#include "TeensyComm.h"


TeensyComm::TeensyComm() {
  messageStart = false;
}

void TeensyComm::begin(long baudRate) {
  Serial.begin(baudRate);
  Serial1.begin(baudRate);
}

void TeensyComm::checkForCommands() {
  while (Serial1.available() > 0) {
    char receivedChar = Serial1.read();
    if (receivedChar == '<') {
      inputBuffer = "";
      messageStart = true;
      continue;
    }

    if (receivedChar == '>' && messageStart) {
      messageStart = false;
      processMessage(inputBuffer);
      inputBuffer = "";
      continue;
    }

    if (messageStart) {
      inputBuffer += receivedChar;
    }
  }
  if (motorActive) {
    digitalWrite(ledPin, HIGH); // Turn LED on
  } else {
    digitalWrite(ledPin, LOW); // Turn LED off
  }
}

void TeensyComm::setLedPin(int pin) {
  ledPin = pin;
  pinMode(ledPin, OUTPUT);
}

void TeensyComm::processMessage(String message) {

  Serial.println("Received message: " + message);

  int separatorIndex = message.lastIndexOf(',');

  String command = message.substring(0, separatorIndex);

  char receivedChecksum = message.charAt(separatorIndex + 1);
  
  if (calculateChecksum(command) == receivedChecksum) {
    if (command == "1") {
      motorActive = true;
    }

  } 
}

char TeensyComm::calculateChecksum(const String &data) {
  char checksum = 0;
  for (unsigned int i = 0; i < data.length(); i++) {
    checksum ^= data[i];
  }
  return checksum;
}

bool TeensyComm::isMotorActivated() {
  return motorActive;
}

void TeensyComm::resetMotorActivation() {
  motorActive = false;
}

bool TeensyComm::isNewCommandAvailable() {
    // Implementation depends on how you track new commands
    return newCommandAvailable;
}

void TeensyComm::clearCommand() {
    newCommandAvailable = false;
}
