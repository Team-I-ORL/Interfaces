#include "MotorPID.h"

MotorPID::MotorPID(double Kp, double Ki, double Kd, int motorPinCW, int motorPinCCW, int encoderPinA, int encoderPinB, double initialSetpoint) 
    : pidController(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT) , myEncoder(encoderPinA, encoderPinB) , motorPinCW(motorPinCW) , motorPinCCW(motorPinCCW) {

    pinMode(motorPinCW, OUTPUT);
    pinMode(motorPinCCW, OUTPUT);
    pidController.SetMode(AUTOMATIC);
    setpoint = initialSetpoint; // Initialize setpoint
}

void MotorPID::compute() {
    if (abs(setpoint - input) < threshold) {
        state = COMPLETED; // Or IDLE, depending on your state logic
        // Stop the motor by setting the output to zero
        output = 0;
        resetEncoder();
    }
    input = myEncoder.read();
    pidController.Compute();
    
    if (state == RUNNING) {
        if (output < 0) {
            analogWrite(motorPinCCW, -output);
            digitalWrite(motorPinCW, LOW);
        } else {
            analogWrite(motorPinCW, output);
            digitalWrite(motorPinCCW, LOW);
        }
        updateState();
    }
    else if (state == IDLE || state == COMPLETED) {
        // Ensure motor remains stopped if not in RUNNING state
        analogWrite(motorPinCW, 0);
        analogWrite(motorPinCCW, 0);
    }

}


void MotorPID::setOutputLimits(double min, double max) {
    pidController.SetOutputLimits(min, max);
}

void MotorPID::initializePID(double Kp, double Ki, double Kd) {
    pidController.SetTunings(Kp, Ki, Kd);
}

void MotorPID::resetEncoder() {
    input = 0; // Reset the input value as well
}

void MotorPID::updateState() {
    if (abs(setpoint - input) < threshold) {
        state = COMPLETED; // Or IDLE, depending on your state logic
        // Stop the motor by setting the output to zero
        output = 0;
        resetEncoder();
    }
}

MotorPID::State MotorPID::getState() const {
    return state;
}

void MotorPID::setState(State newState) {
    state = newState;
    if (state == IDLE) {
        resetEncoder(); // Optional: Reset encoder when motor is idle
    }
}