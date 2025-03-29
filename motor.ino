class Motor {
private:
  int pwm_right, direction_right, pwm_left, direction_left;
  int Forward, Reverse;

public:
  Motor(int pwm_right_pin, int direction_right_pin, int pwm_left_pin, int direction_left_pin) {
    pwm_right = pwm_right_pin;
    direction_right = direction_right_pin;
    pwm_left = pwm_left_pin;
    direction_left = direction_left_pin;
    Forward = 1;
    Reverse = 0;
  }

  void right_turn() {

    digitalWrite(direction_right, Reverse);
    digitalWrite(direction_left, Forward);
    analogWrite(pwm_right, 0);
    analogWrite(pwm_left, 255);
  }

  void left_turn() {
    digitalWrite(direction_right, Forward);
    digitalWrite(direction_left, Reverse);
    analogWrite(pwm_right, 255);
    analogWrite(pwm_left, 0);
  }

  void forward() {
    digitalWrite(direction_right, Forward);
    digitalWrite(direction_left, Forward);
    analogWrite(pwm_right, 0);
    analogWrite(pwm_left, 0);
  }

  void stop() {
    digitalWrite(direction_right, Forward);
    digitalWrite(direction_left, Forward);
    analogWrite(pwm_right, 255);
    analogWrite(pwm_left, 255);
  }
};

Motor buggy(9, 5, 10, 6);

void setup() {
  pinMode(9, OUTPUT);   // pwm_left
  pinMode(5, OUTPUT);   // direction_left
  pinMode(10, OUTPUT);  // pwm_right
  pinMode(6, OUTPUT);   // direction_right
}

void loop() {
}


