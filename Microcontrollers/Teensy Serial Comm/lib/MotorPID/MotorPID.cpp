#include "MotorPID.h"

MotorPID::MotorPID(double Kp, double Ki, double Kd, int motorPinCW, int motorPinCCW, int encoderPinA, int encoderPinB, double initialSetpoint) 
    : pidController(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT) , myEncoder(encoderPinA, encoderPinB) , motorPinCW(motorPinCW) , motorPinCCW(motorPinCCW) {

    pinMode(motorPinCW, OUTPUT);
    pinMode(motorPinCCW, OUTPUT);
    pidController.SetMode(AUTOMATIC);
    setpoint = initialSetpoint; // Initialize setpoint
}

void MotorPID::compute() {

    if (motorActive) {
        input = myEncoder.read();
        pidController.Compute();
        if (output < 0) {
            analogWrite(motorPinCCW, -output);
            digitalWrite(motorPinCW, LOW);
        } else {
            analogWrite(motorPinCW, output);
            digitalWrite(motorPinCCW, LOW);
        }
    }
    else {
        // Ensure motor remains stopped if not in RUNNING state
        analogWrite(motorPinCW, 0);
        analogWrite(motorPinCCW, 0);
        resetEncoder();
    }
    updateState();
    Serial.println("Output: " + String(output));
    Serial.println("Input: " + String(input));

}


void MotorPID::setOutputLimits(double min, double max) {
    pidController.SetOutputLimits(min, max);
}

void MotorPID::initializePID(double Kp, double Ki, double Kd) {
    pidController.SetTunings(Kp, Ki, Kd);
}

void MotorPID::resetEncoder() {
    myEncoder.write(0);

}

void MotorPID::updateState() {
    if (abs(output) < threshold) {
        motorActive = false;
      //  motorActive = false;
    }

}
