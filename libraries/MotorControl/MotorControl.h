#ifndef LIBRARIES_MOTORCONTROL_H
#define LIBRARIES_MOTORCONTROL_H

#include <Arduino.h>
#include <InterruptCounter.h>

//file for motor controller and motor classes

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

            initialized = true;

            //set the direction to forward by default
            set_direction(true);
        }else{
            Serial.println("Right Motor already initialized");
        }
    }
};


class LeftMotor: public Motor{
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
            Serial.println("Left Motor already initialized");
        }
    }
};


/*
class Motor_old{
private:
    int enablePin;
    int directionPin;
    int feedbackPin;
    boolean initialized;
    //unsigned long pulseCount;

    //function called when a rising edge is detected on the feedback pin to
    // count the number of pulses

    //static void count_pulse(){
    //    pulseCount++;
    //}

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


            //pinMode(feedbackPin, INPUT_PULLUP);
            //create interrupt for feedback pin to count pulses
            //attachInterrupt(digitalPinToInterrupt(feedbackPin), count_pulse, RISING);

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



#endif //LIBRARIES_MOTORCONTROL_H
