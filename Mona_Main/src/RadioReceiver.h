#ifndef RADIO_RECEIVER_H
#define RADIO_RECEIVER_H

#include <SPI.h>
#include "RF24.h"

class RadioReceiver {
private:
  RF24 radio;
  uint8_t address[2][6] = { "1Node", "2Node" };
  bool radioNumber;
  float payload;
  int encoderValue;  // Added to store encoder value

public:
  // Constructor
  RadioReceiver(uint8_t cePin = 7, uint8_t csnPin = 8, bool nodeNumber = 0);
  
  // Methods
  bool begin();
  void process();
  int getEncoderVal();  // New method to return encoder value

private:
  void receive();
};

#endif // RADIO_RECEIVER_H