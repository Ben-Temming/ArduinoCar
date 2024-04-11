
//Servo demo (works)
#include <ArduinoCarSensors.h> 

int servo_pin = 5; //2; //4; 
CustomServo my_servo; 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //setup the servo 
  my_servo.setup_servo(servo_pin, 180); 

}

void loop() {
  //test out servo 
  Serial.println("Sevor 0");
  my_servo.move_to(0); 
  delay(2000); 

  Serial.println("Sevor 90");
  my_servo.move_to(90); 
  delay(2000);  
  
  Serial.println("Sevor 180");
  my_servo.move_to(180);  
  delay(2000); 

  // //test servo sweep 
  // Serial.println("Servo Sweep:"); 
  // //reset the servo position 
  // my_servo.move_to(0); 
  // delay(2000); 

  // for (int i=0; i <=180; i+=5){
  //   //step the servo 
  //   Serial.println(i);
  //   my_servo.move_to(i); 
  //   delay(250); 
  // }
  // delay(2000); 
  
}