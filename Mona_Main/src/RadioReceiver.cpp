#include "RadioReceiver.h"
#include "Arduino.h"

RadioReceiver::RadioReceiver(uint8_t cePin, uint8_t csnPin, bool nodeNumber) 
  : radio(cePin, csnPin), 
    radioNumber(nodeNumber),
    payload(0.0),
    encoderValue(0)  // Initialize encoder value
{
}

bool RadioReceiver::begin() {
  // Initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("Radio hardware is not responding!"));
    return false;
  }

  // Set the PA Level low to prevent power supply related problems
  radio.setPALevel(RF24_PA_LOW);

  // Set payload size to size of float
  radio.setPayloadSize(sizeof(payload));

  // Set up the pipes
  radio.openReadingPipe(1, address[!radioNumber]);
  
  // Start in listening mode
  radio.startListening();
  
  Serial.println(F("Radio initialized as dedicated receiver."));
  return true;
}

void RadioReceiver::process() {
  receive();
}

void RadioReceiver::receive() {
  uint8_t pipe;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize();
    radio.read(&payload, bytes);
    
    // Assuming payload contains encoder value, cast it to int
    encoderValue = (int)payload;
    
    Serial.print(F("Received "));
    Serial.print(bytes);
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);
    Serial.print(F(": "));
    Serial.println(payload);
  }
}

int RadioReceiver::getEncoderVal() {
  // Return the stored encoder value
  return encoderValue;
}