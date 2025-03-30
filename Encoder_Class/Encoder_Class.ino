class Encoder {
private:
    int pin;
    volatile int count;
    float speed;
    unsigned long lastUpdateTime;
    static constexpr float wheelCircumference = 3.14159 * 3.0; // 30mm diameter in cm
    static constexpr int ppr = 250;
    static constexpr unsigned long updateInterval = 100; // Timeframe in ms

public:
    Encoder(int encoderPin) {
        pin = encoderPin;
        count = 0;
        speed = 0;
        lastUpdateTime = millis();
        pinMode(pin, INPUT_PULLUP);
    }

    void update() {
        count++;
    }

    void calculateSpeed() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= updateInterval) {
            speed = (count / (float)ppr) * wheelCircumference / (updateInterval / 1000.0);
            count = 0; // Reset count for the next timeframe
            lastUpdateTime = currentTime;
        }
    }

    float getSpeed() {
        return speed;
    }
};

class Motor {
private:
    int pwmPin;
    int dirPin;

public:
    Motor(int pwm, int dir) {
        pwmPin = pwm;
        dirPin = dir;
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    void setSpeed(int speed) {
        if (speed >= 0) {
            digitalWrite(dirPin, LOW); // Forward
            analogWrite(pwmPin, speed);
        } else {
            digitalWrite(dirPin, HIGH); // Backward
            analogWrite(pwmPin, -speed);
        }
    }
};

Encoder leftEncoder(2);
Encoder rightEncoder(3);
Motor leftMotor(10, 5);
Motor rightMotor(9, 6);

void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
}

void loop() {
    if (digitalRead(2) == LOW) leftEncoder.update();
    if (digitalRead(3) == LOW) rightEncoder.update();

    leftEncoder.calculateSpeed();
    rightEncoder.calculateSpeed();

    Serial.print("Left Speed: ");
    Serial.print(leftEncoder.getSpeed());
    Serial.print(" cm/s | Right Speed: ");
    Serial.println(rightEncoder.getSpeed());
    
    delay(10); // Shorter delay for more frequent updates
}
