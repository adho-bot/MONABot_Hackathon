#include "src/RadioReceiver.h"

RadioReceiver radioReceiver;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait for serial connection
  }
  
  Serial.println(F("RF24 Node - Dedicated Receiver Implementation"));
  
  // Create and initialize radio with selected node number - object initialisation
  radioReceiver = RadioReceiver(7, 8, 0);
  
  if (!radioReceiver.begin()) {
    while (1) {} // Hold in infinite loop if radio fails
  }
}

void loop() {
  // Process radio operations (receive only)
  radioReceiver.process();

  //return encoder value
  radioReceiver.getEncoderVal();
}
