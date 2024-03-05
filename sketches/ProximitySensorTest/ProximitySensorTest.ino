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
