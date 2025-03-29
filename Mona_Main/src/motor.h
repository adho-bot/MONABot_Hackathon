#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  int pwm_right, direction_right, pwm_left, direction_left;
  int Forward, Reverse;

public:
  Motor(int pwm_right_pin, int direction_right_pin, int pwm_left_pin, int direction_left_pin);

  void right_turn();
  void left_turn();
  void forward();
  void stop();
};

#endif