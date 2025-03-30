#include "Motor.h"
#include "Encoder.h"

Motor::Motor(int pwm_right_pin, int direction_right_pin,
             int pwm_left_pin, int direction_left_pin,
             Encoder* rightEnc, Encoder* leftEnc) {

    pwm_right = pwm_right_pin;
    direction_right = direction_right_pin;
    pwm_left = pwm_left_pin;
    direction_left = direction_left_pin;

    Forward = 1;
    Reverse = 0;

    Kp = 1.3;
    Ki = 0.0;
    Kd = 0.0;
    targetSpeed = 0;
    errorSum = 0;
    lastError = 0;

    rightEncoder = rightEnc;
    leftEncoder = leftEnc;
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
  // Ensure encoders are updated before reading speed
  rightEncoder->calculateSpeed();
  leftEncoder->calculateSpeed();

  float rightSpeed = rightEncoder->getSpeed();
  float leftSpeed = leftEncoder->getSpeed();

  // Serial.print("Left Speed: ");
  // Serial.print(leftEncoder->getSpeed());
  // Serial.print(" cm/s | Right Speed: ");
  // Serial.println(rightEncoder->getSpeed());

  float errorRight = targetSpeed - rightSpeed;
  float errorLeft = targetSpeed - leftSpeed;

  int pwmRight = computePWMFromSpeed(Kp * errorRight) + 5;
  int pwmLeft = computePWMFromSpeed(Kp * errorLeft) -3 ;
  if(pwmLeft < 0){
    pwmLeft = 0;
  }

  // Serial.print("Left PWM: ");
  // Serial.print(pwmLeft);
  // Serial.print(" cm/s | Right PWM: ");
  // Serial.println(pwmRight);

  digitalWrite(direction_right, Forward);
  digitalWrite(direction_left, Forward);

  analogWrite(pwm_right, pwmRight);
  analogWrite(pwm_left, pwmLeft);
}


void Motor::stop() {
  digitalWrite(direction_right, Forward);
  digitalWrite(direction_left, Forward);
  analogWrite(pwm_right, 255);
  analogWrite(pwm_left, 255);
}

int Motor::computePWMFromSpeed(float pid_output) {
  return constrain(pid_output, 0, 255);
}

void Motor::setTargetSpeed(float speed) {
  targetSpeed = speed;
}
