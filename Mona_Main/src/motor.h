#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Encoder; // Forward declaration

class Motor {
private:
    int pwm_right, direction_right;
    int pwm_left, direction_left;
    int Forward, Reverse;

    float Kp, Ki, Kd;
    float errorSum, lastError;
    int targetSpeed;

    Encoder* rightEncoder;
    Encoder* leftEncoder;

    int computePWMFromSpeed(float speed);

public:
    Motor(int pwm_right_pin, int direction_right_pin,
          int pwm_left_pin, int direction_left_pin,
          Encoder* rightEnc, Encoder* leftEnc);

    void right_turn();
    void left_turn();
    void forward();
    void stop();
    void setTargetSpeed(float speed);
};

#endif
