#include "Encoder.h"

Encoder::Encoder(int encoderPin) {
    pin = encoderPin;
    count = 0;
    countD = 0;
    speed = 0;
    Distance = 0;
    lastUpdateTime = millis();
    pinMode(pin, INPUT_PULLUP);
}

float Encoder::getDistance() {
    Distance = countD * (0.03 * 3.14159 / 1720);
    return Distance;
}

void Encoder::DistanceReset(){
    Distance = 0;
    reset();
}

float Encoder::getCounts() {
    return countD;
}

void Encoder::update() {
    count++;
    countD++;
}

void Encoder::reset() {
    countD = 0;
}

void Encoder::calculateSpeed() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;
    if (elapsedTime > 0) {
        // Calculate speed in cm/s
        speed = (count / (float)ppr) * wheelCircumference / (elapsedTime / 1000.0);
        count = 0; // Reset count for the next timeframe
        lastUpdateTime = currentTime;
    }
}

float Encoder::getSpeed() {
    return speed;
}
