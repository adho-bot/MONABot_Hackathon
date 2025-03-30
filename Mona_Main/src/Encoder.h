#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
private:
    int pin;
    float Distance;
    volatile int count;
    volatile int countD;
    float speed;
    unsigned long lastUpdateTime;
    static constexpr float wheelCircumference = 3.14159 * 3.0; // 30mm diameter in cm
    static constexpr int ppr = 1720;

public:
    Encoder(int encoderPin);
    
    float getDistance();
    void DistanceReset();
    float getCounts();
    void update();
    void reset();
    void calculateSpeed();
    float getSpeed();
};

#endif // ENCODER_H
