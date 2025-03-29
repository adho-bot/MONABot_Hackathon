class Encoder {
private:
    int pin;
    volatile int count;
    float speed;
    unsigned long lastUpdateTime;
    static constexpr float wheelCircumference = 3.14159 * 3.0; // 30mm diameter in cm
    static constexpr int ppr = 250;

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
        unsigned long elapsedTime = currentTime - lastUpdateTime;
        if (elapsedTime > 0) {
            // Calculate speed in cm/s
            speed = (count / (float)ppr) * wheelCircumference / (elapsedTime / 1000.0);
            count = 0; // Reset count for the next timeframe
            lastUpdateTime = currentTime;
        }
    }

    float getSpeed() {
        return speed;
    }
};

Encoder leftEncoder(2);
Encoder rightEncoder(3);

void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), [] { leftEncoder.update(); }, FALLING);
    attachInterrupt(digitalPinToInterrupt(3), [] { rightEncoder.update(); }, FALLING);
}

void loop() {
    leftEncoder.calculateSpeed();
    rightEncoder.calculateSpeed();

    Serial.print("Left Speed: ");
    Serial.print(leftEncoder.getSpeed());
    Serial.print(" cm/s | Right Speed: ");
    Serial.println(rightEncoder.getSpeed());
    
    delay(100); // Adjust delay as needed for your application
}