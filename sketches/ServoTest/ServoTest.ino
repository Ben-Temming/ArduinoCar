
//Servo demo (works)
#include <ArduinoCarSensors.h> 

int servo_pin = 5; //2; //4; 
CustomServo my_servo; 


int analogPin = A1; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
int analog_val = 0; 



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //setup the servo 
  my_servo.setup_servo(servo_pin, 270); 

}

void loop() {
  //test out servo 
  Serial.println("Sevor 0");
  my_servo.move_to(0); 
  analog_val = analogRead(analogPin);  // read the input pin
  Serial.println(analog_val);   
  delay(2000); 
  analog_val = analogRead(analogPin);  // read the input pin
  Serial.println(analog_val);          // debug value
  Serial.println("Sevor 90");
  my_servo.move_to(95); 
  analog_val = analogRead(analogPin);  // read the input pin
  Serial.println(analog_val);   
  delay(2000);  
  analog_val = analogRead(analogPin);  // read the input pin
  Serial.println(analog_val);          // debug value
  Serial.println("Sevor 180");
  my_servo.move_to(200);
  analog_val = analogRead(analogPin);  // read the input pin
  Serial.println(analog_val);   
  delay(2000); 
  analog_val = analogRead(analogPin);  // read the input pin
  Serial.println(analog_val);          // debug value

  // //test servo sweep 
  // Serial.println("Servo Sweep:"); 

  // //reset the servo position 
  // my_servo.move_to(0); 
  // delay(2000); 

  // for (int i=0; i <=200; i+=5){
  //   //step the servo 
  //   Serial.println(i);
  //   my_servo.move_to(i); 
  //   delay(250); 
  // }
  // //my_servo.move_to(270); 
  // delay(2000); 
  
}