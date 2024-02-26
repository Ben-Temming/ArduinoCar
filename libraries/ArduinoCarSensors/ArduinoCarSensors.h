//
// Created by bensa on 21/02/2024.
//

#ifndef LIBRARIES_ARDUINOCARSENSORS_H
#define LIBRARIES_ARDUINOCARSENSORS_H

#include <Arduino.h>
#include <Servo.h>


class ProximitySensor{
protected:
    //store all the values needed for the proximity unit
    int trigger_pin;
    int echo_pin;
    boolean initialized;

public:
    ProximitySensor(){
        initialized = false;
    }

    void setup_proximity_sensor(int trig_pin, int ech_pin){
        if (!initialized){
            //store the pin values
            trigger_pin = trig_pin;
            echo_pin = ech_pin;
            //setup the pins
            pinMode(trigger_pin, OUTPUT); //set the trigger_pin as an Output
            pinMode(echo_pin, INPUT); //set the trigger_pin as an Input
            initialized = true;
        }
    }


    void measure_distance(int& measured_dist_cm){
        if (initialized){
            //clear the trigger pin
            digitalWrite(trigger_pin, LOW);
            delayMicroseconds(2);
            //set the trigger pin on HIGH state for 10 mirco seconds
            digitalWrite(trigger_pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(trigger_pin, LOW);
            // Reads the echo pin, returns the sound wave travel time in microseconds
            long duration = pulseIn(echo_pin, HIGH);
            // Calculating the distance (in cm)
            // time * speed of sound / 2
            measured_dist_cm = duration * 0.034 / 2;

            //set the measured distance
            //measured_dist_cm = distance;
        }else{
            Serial.println("Not initialized");
        }
    }
};


class CustomServo{
private:
    //create servo object
    Servo my_servo;
    boolean initialized;
    int max_angle;
public:
    CustomServo(){
        initialized = false;
    }

    void setup_servo(int ser_pin, int max_ang){
        if (!initialized) {
            //attach servo to the servo pin
            my_servo.attach(ser_pin);
            max_angle = max_ang;
            initialized = true;
        }
    }


    void move_to(int angle){
        if (initialized) {
            //the angle has to be in the range from 0 to max_angle degree
            if (angle < 0){
                angle = 0;
            }
            if (angle >= max_angle){
                angle = max_angle;
            }

            //map the angle to 0 to 180 as this is the pulse width that controls the servo
            int servo_val = map(angle, 0, max_angle, 0, 180);

            //set the servo
            my_servo.write(servo_val);

            /*
            //has to be in the range of 0 to 180 degree
            if (angle >= 0 && angle <= 400) {
                my_servo.write(angle);
            } else {
                Serial.println("Invalid angle. Has to be in range 0 to 180 degree");
            }*/
        }
    }
};







#endif //LIBRARIES_ARDUINOCARSENSORS_H
