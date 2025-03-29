#include "Motor.h"

Motor::Motor(int pwm_right_pin, int direction_right_pin, int pwm_left_pin, int direction_left_pin) {
  pwm_right = pwm_right_pin;
  direction_right = direction_right_pin;
  pwm_left = pwm_left_pin;
  direction_left = direction_left_pin;
  Forward = 1;
  Reverse = 0;
}

void Motor::right_turn() {
  digitalWrite(direction_right, Reverse);
  digitalWrite(direction_left, Forward);
  analogWrite(pwm_right, 0);
  analogWrite(pwm_left, 255);
}

void Motor::left_turn() {
  digitalWrite(direction_right, Forward);
  digitalWrite(direction_left, Reverse);
  analogWrite(pwm_right, 255);
  analogWrite(pwm_left, 0);
}

void Motor::forward() {
  digitalWrite(direction_right, Forward);
  digitalWrite(direction_left, Forward);
  analogWrite(pwm_right, 0);
  analogWrite(pwm_left, 0);
}

void Motor::stop() {
  digitalWrite(direction_right, Forward);
  digitalWrite(direction_left, Forward);
  analogWrite(pwm_right, 255);
  analogWrite(pwm_left, 255);
}