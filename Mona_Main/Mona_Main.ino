#include "src/motor.h"
#include "src/Encoder.h"
#include "src/RadioReceiver.h"


Encoder leftEncoder(2);
Encoder rightEncoder(3);
Motor buggy(9, 5, 10, 6, &leftEncoder, &rightEncoder);


RadioReceiver radioReceiver;
typedef enum {Forward, Left, Right, Stopped} Motion;
Motion state;
Motion last_state;

void DriveDistance(float distance){
  leftEncoder.DistanceReset();
  rightEncoder.DistanceReset();
  float combDistance = 0;
  buggy.forward();
  while(combDistance < distance){
    combDistance = (leftEncoder.getDistance()+rightEncoder.getDistance())/2;
    delay(10);
  }
  buggy.stop();
}

void turnUntilAngle(float targetAngle, bool turnRight) {

    // Constants for calculation
    const float wheelDiameter = 0.03;  // 30 mm = 0.03 m
    const float wheelCircumference = 3.14159 * wheelDiameter; // C = π * D
    const int encoderTicksPerRev = 1700; // Encoder resolution (ticks per revolution)
    const float wheelbase = 0.078; // Distance between wheels in meters

    // Calculate the required encoder ticks using the correct formula
    int targetTicks = round(((targetAngle * 3.14159f / 180.0f) * (wheelbase / 2.0f) / wheelCircumference) * encoderTicksPerRev);


    // Reset encoders before starting the turn
    leftEncoder.reset();
    rightEncoder.reset();

    // Start turning in the specified direction
    if (turnRight) {
        buggy.right_turn();
    } else {
        buggy.left_turn();
    }

    // Keep turning until the target number of ticks is reached
    while (abs(leftEncoder.getCounts()) < targetTicks && abs(rightEncoder.getCounts()) < targetTicks) {
        // Continuously check the encoder values
    }

    // Stop the robot once the turn is complete
    buggy.stop();
}

void StateMachine(float parameter){
  switch (state) {             
            case (Forward) :  
                parameter = parameter /1000;
                DriveDistance(parameter);                                     
                break;                          
            case (Left) :
                turnUntilAngle(parameter, false);                                        
                break;                     
            case (Right) : 
                turnUntilAngle(parameter, true);                                   
                break; 
            case (Stopped) :
                buggy.stop();
                break;                           
            default :  
              buggy.stop();    
  }                     
}

int radio_interperet(int signal){
  int first_value = signal / 10; // angle or distance it has to mo ve
  int second_value = signal % 10; // which function it is doing
  if(second_value == 1){
    state = Forward;
  }
  else if(second_value == 2){
    state = Left;
  }
  else if(second_value == 3){
    state = Right;
  }
  else if(second_value == 4){
    state = Stopped;
  }
  return first_value;
}

int radio_val = 0;
float parameter = 0;
float last_radio_val = 0;


void setup() {
  pinMode(9, OUTPUT);  // pwm_left
  pinMode(5, OUTPUT);  // dir_left
  pinMode(10, OUTPUT); // pwm_right
  pinMode(6, OUTPUT);  // dir_right
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), [] { leftEncoder.update(); }, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), [] { rightEncoder.update(); }, FALLING);
  buggy.setTargetSpeed(10);  
    while (!Serial) {
    // Wait for serial connection
  }
  
  Serial.println(F("RF24 Node - Dedicated Receiver Implementation"));
  
  // Create and initialize radio with selected node number - object initialisation
  radioReceiver = RadioReceiver(7, 8, 0);
  
  if (!radioReceiver.begin()) {
    while (1) {} // Hold in infinite loop if radio fails
  }
  state = Stopped;
}


void loop() {
  radioReceiver.process();
  radio_val = radioReceiver.getEncoderVal();

  if (radio_val != last_radio_val) {  // New command received
    parameter = radio_interperet(radio_val);  // Updates `state` as well
    StateMachine(parameter);  // Execute the motion
    last_radio_val = radio_val;  // Update last command
  }

  // You can still log speeds here if needed
}