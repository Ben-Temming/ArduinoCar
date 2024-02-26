
#ifndef LIBRARIES_INTERRUPTCOUNTER_H
#define LIBRARIES_INTERRUPTCOUNTER_H

#include <Arduino.h>


// Define the base class for handling interrupts
class InterruptBase {
public:
    //declare the common functions as virtual
    virtual void InterServ() = 0;
    virtual void setup_interrupt_counter(int inter_pin) = 0;
    virtual unsigned long get_counter_value() = 0;
    virtual void reset_counter() = 0;
};


// The table below shows the available interrupt pins Mega2560.
//	    interrupt:	int.0	int.1	int.2	int.3	int.4	int.5
//   		2		3		21		20		19		18
const int MAX_INTERRUPTS=6;
enum ArduinoInterruptNames{
    INT_DEV1=0,
    INT_DEV2,
    INT_DEV3,
    INT_DEV4,
    INT_DEV5,
    INT_DEV6,
    INT_MAX
};

//pointers to different InterruptBase objects
static InterruptBase *InterruptInstances[MAX_INTERRUPTS];


//Class to handle the interrupts for counting
class InterruptCounter1: public InterruptBase{
private:
    unsigned long counter;
    boolean initialized;
    int interruptPin;

public:
    InterruptCounter(){
        initialized = false;
        counter = 0;
    }

    void setup_interrupt_counter(int inter_pin) override{
        if (!initialized){
            interruptPin = inter_pin;

            //store the object in the interrupt instances array
            InterruptInstances[INT_DEV1] = this;

            // Attach interrupt for counting
            pinMode(interruptPin, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(interruptPin), ISRFunc, RISING);

            initialized = true;
        }else{
            Serial.println("Interrupt counter is already initialized");
        }
    }

    //Function to handle the interrupt and increment the counter
    virtual void InterServ() override{
        counter++;
    }

    // Static function to forward the interrupt to the appropriate instance
    static void ISRFunc() {
        InterruptInstances[INT_DEV1]->InterServ();
    }

    //getter function for counter
    unsigned long get_counter_value() override {
        if (initialized) {
            return counter;
        }else{
            Serial.println("Interrupt counter not initialized");
            return 0;
        }
    }

    //reset function
    void reset_counter() override{
        if (initialized) {
            counter = 0;
        }else{
            Serial.println("Interrupt counter not initialized");
        }
    }
};



class InterruptCounter2: public InterruptBase{
private:
    unsigned long counter;
    boolean initialized;
    int interruptPin;

public:
    InterruptCounter(){
        initialized = false;
        counter = 0;
    }

    void setup_interrupt_counter(int inter_pin) override{
        if (!initialized){
            interruptPin = inter_pin;

            //store the object in the interrupt instances array
            InterruptInstances[INT_DEV2] = this;

            // Attach interrupt for counting
            pinMode(interruptPin, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(interruptPin), ISRFunc, RISING);

            initialized = true;
        }else{
            Serial.println("Interrupt counter is already initialized");
        }
    }

    //Function to handle the interrupt and increment the counter
    virtual void InterServ() override{
        counter++;
    }

    // Static function to forward the interrupt to the appropriate instance
    static void ISRFunc() {
        InterruptInstances[INT_DEV2]->InterServ();
    }

    //getter function for counter
    unsigned long get_counter_value() override {
        if (initialized) {
            return counter;
        }else{
            Serial.println("Interrupt counter not initialized");
            return 0;
        }
    }

    //reset function
    void reset_counter() override{
        if (initialized) {
            counter = 0;
        }else{
            Serial.println("Interrupt counter not initialized");
        }
    }
};


#endif //LIBRARIES_INTERRUPTCOUNTER_H
