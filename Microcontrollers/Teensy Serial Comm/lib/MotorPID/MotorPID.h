#ifndef MotorPID_h
#define MotorPID_h

#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h> // Include Encoder library

class MotorPID {
public:
    enum State { IDLE, RUNNING, COMPLETED };
    MotorPID(double Kp, double Ki, double Kd, int motorPinCW, int motorPinCCW, int encoderPinA, int encoderPinB, double initialSetpoint);
    void compute();
    void setOutputLimits(double min, double max);
    void setState(State newState); // Method to set the state externally
    void resetEncoder();
    void updateState(); // Update the state of the motor
    State getState() const; // Get the current state

private:
    PID pidController;
    Encoder myEncoder;
    int motorPinCW; // Pin for Clockwise rotation
    int motorPinCCW; // Pin for Counter-Clockwise rotation
    double setpoint; // Desired value
    double output; // Control output
    double input; // Measured value from encoder
    State state; // Current state of the motor
    const double threshold = 10; // Threshold for completion

    void initializePID(double Kp, double Ki, double Kd);
};

#endif
