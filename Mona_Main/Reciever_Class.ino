//node number 0,1

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

class RadioReceiver {
private:
  RF24 radio;
  uint8_t address[2][6] = { "1Node", "2Node" };
  bool radioNumber;
  float payload;

public:
  // Constructor automatically sets up as receiver
  RadioReceiver(uint8_t cePin = 7, uint8_t csnPin = 8, bool nodeNumber = 0) 
    : radio(cePin, csnPin), 
      radioNumber(nodeNumber),
      payload(0.0)
  {
  }

  bool begin() {
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

  void process() {
    receive();
  }

  float getEncoderVal(){
    return payload;
  }

private:
  void receive() {
    uint8_t pipe;
    if (radio.available(&pipe)) {
      uint8_t bytes = radio.getPayloadSize();
      radio.read(&payload, bytes);
      Serial.print(F("Received "));
      Serial.print(bytes);
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);
      Serial.print(F(": "));
      Serial.println(payload);
    }
  }
};

// Global instance
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