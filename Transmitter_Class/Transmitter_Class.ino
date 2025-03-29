#include <SPI.h>
#include "printf.h"
#include "RF24.h"

class NRF24Transmitter {
public:
    NRF24Transmitter(uint8_t cePin, uint8_t csnPin, uint8_t radioNumber) 
        : radio(cePin, csnPin), radioNumber(radioNumber), payload(0.0), hasNewPayload(false) {}

    void begin() {
        Serial.begin(115200);
        while (!Serial) {
            // wait for serial port to connect
        }

        if (!radio.begin()) {
            Serial.println(F("radio hardware is not responding!!"));
            while (1) {}
        }

        Serial.println(F("RF24/examples/GettingStarted"));
        Serial.print(F("radioNumber = "));
        Serial.println((int)radioNumber);

        radio.setPALevel(RF24_PA_LOW);
        radio.setPayloadSize(sizeof(payload));
        radio.openWritingPipe(address[radioNumber]);
        radio.openReadingPipe(1, address[!radioNumber]);

        radio.stopListening(); // Transmitter mode
    }

    void transmit() {
        if (!hasNewPayload) return; // Only transmit if there's new data
        
        unsigned long start_timer = micros();
        bool report = radio.write(&payload, sizeof(float));
        unsigned long end_timer = micros();

        if (report) {
            Serial.print(F("Transmission successful! "));
            Serial.print(F("Time to transmit = "));
            Serial.print(end_timer - start_timer);
            Serial.print(F(" us. Sent: "));
            Serial.println(payload);
        } else {
            Serial.println(F("Transmission failed or timed out"));
        }

        hasNewPayload = false; // Reset flag after sending
    }

    void editPayload(float encoder_val) {
        payload = encoder_val;
        hasNewPayload = true;
    }

private:
    RF24 radio;
    uint8_t radioNumber;
    float payload;
    bool hasNewPayload;
    uint8_t address[2][6] = { "1Node", "2Node" };
};

NRF24Transmitter transmitter(9, 10, 1); // using pin 9 for CE, 10 for CSN, and radio number 1

void setup() {
    transmitter.begin();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        float newPayload = input.toFloat();
        transmitter.editPayload(newPayload);
        Serial.print(F("Payload updated to: "));
        Serial.println(newPayload);
    }
    
    transmitter.transmit();
}