//Test left and right motor at the same Time
/*make github repo for arduino car (track stuff on git*/


///* workds
//test left motor class (inheritance from motor)
#include <MotorControl.h> 

const int enablePin = 10;  
const int directionPin = 11;  
const int feedbackPin = 3;

//create the motor object 

RightMotor my_RightMotor; 

void setup(){
  Serial.begin(9600);

  //setup motor 
  my_RightMotor.setup_motor(enablePin, directionPin, feedbackPin); 

}

void loop(){
  //test the motor (should turn the wheel 1 revolution)
  my_RightMotor.test_motor(); 

  //wait 10 seconds
  delay(1000); 
}//*/

/*
//interrutp test 
#include <InterruptCounter.h> 

//note (conter only works when wheel is spinning)
const int feedbackPin = 3;

InterruptCounter1 my_interruptCounter;


const int enablePin = 10;  // Replace with your actual pin number
const int directionPin = 11;  // Replace with your actual pin number


void setup(){
  Serial.begin(9600);

  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);

  my_interruptCounter.setup_interrupt_counter(feedbackPin); 

  digitalWrite(enablePin, HIGH);  // Enable the motor
  digitalWrite(directionPin, HIGH);  // Set direction to forward
}

void loop(){
  
  unsigned long counter = my_interruptCounter.get_counter_value(); 

  if (counter != 0 && counter % 40 == 0){
    Serial.print("Revolution, counter: "); 
    Serial.println(counter); 
  }
  

  //wait 10 seconds
  //delay(1000); 
}//*/


/*
//test motor class 
#include <ArduinoCarSensors.h> 

const int enablePin = 10;  
const int directionPin = 11;  
const int feedbackPin = 3;

//create the motor object 
Motor_old my_motor; 

void setup(){
  Serial.begin(9600);

  //setup motor 
  my_motor.setup_motor(enablePin, directionPin, feedbackPin); 

}

void loop(){
  //test the motor (should turn the wheel 1 revolution)
  my_motor.test_motor(); 

  //wait 10 seconds
  delay(1000); 
}//*/

/*
//motor control

#include <Arduino.h>

const int enablePin = 10;  // Replace with your actual pin number
const int directionPin = 11;  // Replace with your actual pin number
const int feedbackPin = 3;  // Replace with your actual pin number

volatile long pulseCount = 0;
unsigned long previousMillis = 0;
const int debounceTime = 10;

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(feedbackPin, INPUT_PULLUP);

  digitalWrite(enablePin, HIGH);  // Enable the motor
  digitalWrite(directionPin, HIGH);  // Set direction to forward

  attachInterrupt(digitalPinToInterrupt(feedbackPin), countPulse, RISING);
  
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  // Add any other code you need to execute in the loop

  if (currentMillis - previousMillis >= debounceTime) {
    // Update every debounceTime milliseconds
    previousMillis = currentMillis;

    // Print the pulse count and reset it to 0
    Serial.print("Revolutions: ");
    Serial.println(pulseCount);
    pulseCount = 0;
  }
}

void countPulse() {
  // This function is called when a rising edge is detected on the feedback pin
  pulseCount++;
}

//*/

/* (works)
#include <ArduinoCarSensors.h> 


const int trigger_pin = 8; 
const int echo_pin = 9;    

ProximitySensor proximitySensor; // Create an object of ProximitySensor


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  proximitySensor.setup_proximity_sensor(trigger_pin, echo_pin); 
}

void loop() {
  // put your main code here, to run repeatedly:

  //measure the distance 
  int dist_cm; 
  proximitySensor.measure_distance(dist_cm); 
  Serial.print("Distance: ");
  Serial.print(dist_cm);
  Serial.println(" cm");
  
  //1 second wait 
  delay(1000); 
 
}
//*/


/*
//Servo demo (works)
#include <ArduinoCarSensors.h> 

int servo_pin = 2; //4; 
CustomServo my_servo; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //setup the servo 
  my_servo.setup_servo(servo_pin, 180); 

}

void loop() {
  //test out servo 
  //Serial.println("Sevor 0");
  //my_servo.move_to(0); 
  //delay(2000); 
  //Serial.println("Sevor 90");
  //my_servo.move_to(90); 
  //delay(2000);  
  //Serial.println("Sevor 180");
  //my_servo.move_to(180);
  //delay(2000); 

  //test servo sweep 
  Serial.println("Servo Sweep:"); 
  for (int i=0; i <=360; i+=5){
    //step the servo 
    Serial.println(i);
    my_servo.move_to(i); 
    delay(50); 
  }
  delay(2000); 
}
//*/


/* basic communcation (works)
#include <Arduino.h>
int x;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  while (!Serial.available());
  x = Serial.readString().toInt();
  Serial.println(x + 1);
}//*/


/*sending multiple values
#include <Arduino.h>
int num1 = 0;
int num2 = 0;
int count = 0; 

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  // Send two numbers separated by a comma and followed by a newline character
  Serial.print(num1);
  Serial.print(",");
  Serial.print(num2);
  Serial.println();

  if (count % 2 == 0){
    num1++; 
  }else{
    num2++;
  }
   
  count++; 

  while (!Serial.available());
  int x = Serial.readString().toInt();
  if (x==1){
    Serial.print(333);
    Serial.print(",");
    Serial.print(333);
    Serial.println();
  }

  delay(1000);  // Adjust the delay based on your needs
}

//*/


