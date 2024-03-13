#include <Arduino.h>
#include "TeensyComm.h"
#include "MotorPID.h"

TeensyComm espComm;

// Define pins for the motor and encoder
const int motorPinCW = 5; // Pin for Clockwise rotation
const int motorPinCCW = 6; // Pin for Counter-Clockwise rotation
const int encoderPinA = 3; // Pin for Encoder A
const int encoderPinB = 4; // Pin for Encoder B
// Define LED pin
const int ledPin = 13;

int setpoint = 180 * 4; // 1 rev setpoint for the motor (180 CPR times 4 for Teensy Encoder Library)

volatile bool motorActive = false;

// Initialize the MotorPID object with PID parameters, motor control pins, encoder pins, and initial setpoint
double Kp=1.2, Kd=0, Ki=0.01; //PI control WITHOUT LOAD FOR NOW TODO: TUNE
MotorPID motorPID(Kp, Ki, Kd, motorPinCW, motorPinCCW, encoderPinA, encoderPinB, setpoint); // Example values

void setup() {
  espComm.begin(9600);
  espComm.setLedPin(ledPin);
  motorPID.resetEncoder();
}

void loop() {
  espComm.checkForCommands();

  // Check if the motor should be activated or deactivated based on the command
  // if (motorActive) {
  //   motorPID.setState(MotorPID::RUNNING);
  //   Serial.println("Motor activated");

  // } else {
  //   motorPID.setState(MotorPID::IDLE);
  //   Serial.println("Motor deactivated");
  // }

  // Continuously compute PID and control motor
  motorPID.compute();

  // // Update motorActive based on MotorPID state
  // if (motorPID.getState() == MotorPID::COMPLETED) {
  //   motorActive = false; // Automatically stop the motor
  // }

  Serial.println(motorActive);
  // Optional: Debug output to monitor motor state
  // static MotorPID::State lastState = MotorPID::IDLE;

  // MotorPID::State currentState = motorPID.getState();
  // if (currentState != lastState) {
  //   lastState = currentState;
  //   Serial.print("Motor State Changed: ");
  //   Serial.println(currentState);
  // }
  delay(200);
}
