#include "TeensyComm.h"

TeensyComm::TeensyComm() {
  messageStart = false;
  activateMotor = false;
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
}

void TeensyComm::setLedPin(int pin) {
  ledPin = pin;
  pinMode(ledPin, OUTPUT);
}

void TeensyComm::processMessage(String message) {
  int separatorIndex = message.lastIndexOf(',');
  String command = message.substring(0, separatorIndex);
  char receivedChecksum = message.charAt(separatorIndex + 1);
  if (calculateChecksum(command) == receivedChecksum) {
    if (command == "1") {
      digitalWrite(ledPin, HIGH); // Turn LED on
      activateMotor = true;
    } else if (command == "0") {
      digitalWrite(ledPin, LOW); // Turn LED off
      activateMotor = false;
    }
  } else {
    // Handle checksum error
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
  return activateMotor;
}

void TeensyComm::resetMotorActivation() {
  activateMotor = false;
}
