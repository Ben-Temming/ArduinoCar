
#include <ArduinoCar.h>

//define all the pins
//servo pin 
const int servo_pin = 5;
//distance measure pins 
const int trigger_pin = 8; 
const int echo_pin = 9; 
//right motor pins 
const int right_enable_pin = 10;  
const int right_direction_pin = 11;  
const int right_feedback_pin = 2;
//left motor pins (change to actual values)
const int left_enable_pin = 12; 
const int left_direction_pin = 13; 
const int left_feedback_pin = 3; 


//create the arduino car object 
ArduinoCar my_arduinoCar; 


void setup() {
  Serial.begin(115200); 

  //setup the arduino car
  my_arduinoCar.setup_arduino_car(servo_pin, trigger_pin, 
                                  echo_pin, left_enable_pin, 
                                  left_direction_pin, left_feedback_pin,
                                  right_enable_pin, right_direction_pin, 
                                  right_feedback_pin);

  //Testing components 
  //my_arduinoCar.test_components(); 

}

void loop() {
  //run the arduino car 
  my_arduinoCar.run(); 


  delay(10000); 

}
