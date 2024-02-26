//
// Created by bensa on 21/02/2024.
//

#ifndef LIBRARIES_ARDUINOCARSENSORS_H
#define LIBRARIES_ARDUINOCARSENSORS_H

#include <Arduino.h>
#include <Servo.h>
#include <InterruptCounter.h>

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


/*
class LeftMotor{
private:
    int enablePin;
    int directionPin;
    int feedbackPin;
    boolean initialized;

    //interrupt counter that can be used to count the number of revolutions
    InterruptCounter1 my_interruptCounter;

public:
    LeftMotor(){
        initialized = false;
    }

    void setup_motor(int en_pin, int dir_pin, int feed_pin){
        if (!initialized){
            enablePin = en_pin;
            directionPin = dir_pin;
            feedbackPin = feed_pin;

            //setup each pin
            pinMode(enablePin, OUTPUT);
            pinMode(directionPin, OUTPUT);

            //setup the interrupt counter
            my_interruptCounter.setup_interrupt_counter(feedbackPin);

            initialized = true;

            //set the direction to forward by default
            set_direction(true);
        }else{
            Serial.println("Motor already initialized");
        }
    }

};*/

class Motor{
public:
    int enablePin;
    int directionPin;
    int feedbackPin;
    boolean initialized;

    //interrupt counter that can be used to count the number of revolutions
    //wil be InterruptCounter1 for right motor and InterruptCounter2 for left motor
    InterruptBase* my_interruptCounter;


    Motor(){
        initialized = false;
    }

    virtual void setup_motor(int en_pin, int dir_pin, int feed_pin) = 0;

    void set_direction(boolean forward=true){
        if (initialized){
            //set the direction of the motor
            if (forward){
                //set direction forward
                digitalWrite(directionPin, HIGH);
            }else{
                //set direction backward
                digitalWrite(directionPin, LOW);
            }
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void enable_motor(){
        if (initialized){
            //enable the motor by setting the enable pin to high
            digitalWrite(enablePin, HIGH);
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void disable_motor(){
        if (initialized){
            //disable the motor by setting the enable pin to low
            digitalWrite(enablePin, LOW);
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void test_motor(){
        if (initialized){
            //Testing

            Serial.println("Moving wheel for 1 second");

            //Trun on motor
            enable_motor();

            //reset the interrupt counter
            my_interruptCounter->reset_counter();

            delay(2000);

            unsigned long counter_val = my_interruptCounter->get_counter_value();
            Serial.print("Counter value: ");
            Serial.println(counter_val);

            disable_motor();
            Serial.println("Finished moving");

        }else{
            Serial.println("Motor not initialized");
        }
    }

};


class RightMotor: public Motor{
//private:
    //interrupt counter that can be used to count the number of revolutions
  //  InterruptCounter1 my_interruptCounter;

public:
    virtual void setup_motor(int en_pin, int dir_pin, int feed_pin){
        if (!initialized){
            enablePin = en_pin;
            directionPin = dir_pin;
            feedbackPin = feed_pin;

            //setup each pin
            pinMode(enablePin, OUTPUT);
            pinMode(directionPin, OUTPUT);

            //create an instance of InterruptCounter1
            InterruptCounter1* myInterruptCounter1 = new InterruptCounter1();
            myInterruptCounter1->setup_interrupt_counter(feedbackPin);
            // Assign the base class pointer to the derived class instance
            my_interruptCounter = myInterruptCounter1;


            //setup the interrupt counter
            //my_interruptCounter.setup_interrupt_counter(feedbackPin);

            initialized = true;

            //set the direction to forward by default
            set_direction(true);
        }else{
            Serial.println("Motor already initialized");
        }
    }
};

class LeftMotor: public Motor{
//private:
    //interrupt counter that can be used to count the number of revolutions
    //  InterruptCounter1 my_interruptCounter;

public:
    virtual void setup_motor(int en_pin, int dir_pin, int feed_pin){
        if (!initialized){
            enablePin = en_pin;
            directionPin = dir_pin;
            feedbackPin = feed_pin;

            //setup each pin
            pinMode(enablePin, OUTPUT);
            pinMode(directionPin, OUTPUT);

            //create an instance of InterruptCounter1
            InterruptCounter2* myInterruptCounter2 = new InterruptCounter2();
            myInterruptCounter2->setup_interrupt_counter(feedbackPin);
            // Assign the base class pointer to the derived class instance
            my_interruptCounter = myInterruptCounter2;

            initialized = true;

            //set the direction to forward by default
            set_direction(true);
        }else{
            Serial.println("Motor already initialized");
        }
    }
};


///*
class Motor_old{
private:
    int enablePin;
    int directionPin;
    int feedbackPin;
    boolean initialized;
    //unsigned long pulseCount;

    //function called when a rising edge is detected on the feedback pin to
    // count the number of pulses
    /*
    static void count_pulse(){
        pulseCount++;
    }*/

public:
    Motor_old(){
        initialized = false;
        //pulseCount = 0;
    }

    void setup_motor(int en_pin, int dir_pin, int feed_pin){
        if (!initialized){
            enablePin = en_pin;
            directionPin = dir_pin;
            feedbackPin = feed_pin;

            //setup each pin
            pinMode(enablePin, OUTPUT);
            pinMode(directionPin, OUTPUT);

            /*
            pinMode(feedbackPin, INPUT_PULLUP);
            //create interrupt for feedback pin to count pulses
            attachInterrupt(digitalPinToInterrupt(feedbackPin), count_pulse, RISING);
             */
            initialized = true;

            //set the direction to forward by default
            set_direction(true);
        }else{
            Serial.println("Motor already initialized");
        }
    }

    void set_direction(boolean forward=true){
        if (initialized){
            //set the direction of the motor
            if (forward){
                //set direction forward
                digitalWrite(directionPin, HIGH);
            }else{
                //set direction backward
                digitalWrite(directionPin, LOW);
            }
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void enable_motor(){
        if (initialized){
            //enable the motor by setting the enable pin to high
            digitalWrite(enablePin, HIGH);
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void disable_motor(){
        if (initialized){
            //disable the motor by setting the enable pin to low
            digitalWrite(enablePin, LOW);
        }else{
            Serial.println("Motor not initialized");
        }
    }


    void test_motor(){
        if (initialized){
            //travel 1 revolution
            Serial.println("Moving wheel 1 revolution");

            int pulsesPerWheelRevolution = 40;

            //reset pulse count
            //pulseCount = 0;
            enable_motor();

            delay(2000);

            //wait while the wheel is turning
            //while (pulseCount < pulsesPerWheelRevolution){
            //    delay(0.01);
            //}

            disable_motor();
            Serial.println("Finished moving");

        }else{
            Serial.println("Motor not initialized");
        }
    }



};
//*/






#endif //LIBRARIES_ARDUINOCARSENSORS_H
