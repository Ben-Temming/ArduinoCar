//test the motor control 
#include <MotorControl.h> 

//right motor pins 
const int right_enablePin = 10;  
const int right_directionPin = 11;  
const int right_feedbackPin = 2;

//left motor pins (change to actual values)
const int left_enablePin = 12; 
const int left_directionPin = 13; 
const int left_feedbackPin = 3; // 21; //3; 

//create the motorcontrol object 
MotorController my_motorController; 


void setup() {
  Serial.begin(9600);

  //setup motor controller 
  my_motorController.setup_motor_controller(left_enablePin, left_directionPin, left_feedbackPin, 
                                            right_enablePin, right_directionPin, right_feedbackPin); 


}

void loop() {
  float actual_dist = my_motorController.move_forward_dist_closed_loop(40.0);
  Serial.println("Set travel dist to 40cm"); 
  Serial.print("Actual dist travelled (cm): "); 
  Serial.println(actual_dist); 
  delay(5000);  
}
