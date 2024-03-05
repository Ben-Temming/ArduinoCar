#ifndef LIBRARIES_MOTORCONTROL_H
#define LIBRARIES_MOTORCONTROL_H

#include <Arduino.h>
#include <math.h>
#include <InterruptCounter.h>

//file for motor controller and motor classes

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


class Motor{
public:
    int enablePin;
    int directionPin;
    int feedbackPin;
    boolean initialized;

    //const int minStartPWMVal = 190; //PWM value where it can start moving
    const int startPWMVal = 200;
    const int maxPWMVal = 255;
    const int minPWMVal = 190;
    int currentPWMVal;

    //interrupt counter that can be used to count the number of revolutions
    //wil be InterruptCounter1 for right motor and InterruptCounter2 for left motor
    InterruptBase* my_interruptCounter;

    Motor(){
        initialized = false;
        currentPWMVal = startPWMVal;
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
            //digitalWrite(enablePin, HIGH);

            //// Set the PWM signal to control the power level
            //min needed for moving 190, to max 255
            analogWrite(enablePin, startPWMVal); //255);
            //update current PWM val
            currentPWMVal = startPWMVal;
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void update_PWMVal(int val){
        if (initialized){
            //check that the value is acceptable
            if (val > maxPWMVal){
                val = maxPWMVal;
            }
            if (val < minPWMVal){
                val = minPWMVal;
            }

            //set the new PWM value
            analogWrite(enablePin, val);

            //update current PWM val
            currentPWMVal = val;
        }
    }

    void lock_motor(){
        if (initialized){
            //set the PWM value low so that it does not turn but is locked
            analogWrite(enablePin, 150);
        }
    }

    int get_current_PWMVal(){
        if (initialized){
            return currentPWMVal;
        }
    }

    void disable_motor(){
        if (initialized){
            //disable the motor by setting the enable pin to low
            digitalWrite(enablePin, LOW);
            //analogWrite(enablePin, 0);
        }else{
            Serial.println("Motor not initialized");
        }
    }

    void reset_interrupt_counter(){
        if (initialized){
            my_interruptCounter->reset_counter();
        }else{
            Serial.println("Motor not initialized");
        }

    }

    unsigned long get_interrupt_counter_value(){
        if (initialized){
            return my_interruptCounter->get_counter_value();
        }else{
            Serial.println("Motor not initialized");
            return 0;
        }
    }


    void test_motor(){
        if (initialized){
            //Testing

            Serial.println("Moving wheel for 1 second");

            //Trun on motor
            enable_motor();

            //reset the interrupt counter
            reset_interrupt_counter();
            //my_interruptCounter->reset_counter();

            delay(2000);

            //unsigned long counter_val = my_interruptCounter->get_counter_value();
            unsigned long counter_val = get_interrupt_counter_value();
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



//Motor controller that uses left and right motor to perform more complex maneuvers
class MotorController{
private:
    boolean initialized;

    //create the objects for the left and right motors
    RightMotor my_rightMotor;
    LeftMotor my_leftMotor;

    const float PULSES_PER_WHEEL_ROTATION = 40.0; //estimate (might have to be changed) (float for float division)
    const float WHEEL_CIRCOMFERENCE_CM = 15.71;
    const float WHEEL_RADIUS = 2.5; //cm
    //calculate distance travled as (pulese/pulese per wheel)*(C =2pi r), assuming wheels dont slip

    // Proportional gain for the closed-loop control
    const float KP = 0.1;  // needs tuning

    //need distance it takes to break (from setting motor to disable to stand still)

public:
    MotorController(){
        initialized = false;
    }

    void setup_motor_controller(int left_en_pin, int left_dir_pin, int left_feed_pin, int right_en_pin, int right_dir_pin, int right_feed_pin){
        if (!initialized){
            //initialize the left motor
            my_leftMotor.setup_motor(left_en_pin, left_dir_pin, left_feed_pin);

            //initialize the right motor
            my_rightMotor.setup_motor(right_en_pin, right_dir_pin, right_feed_pin);

            initialized = true;
        }else{
            Serial.println("Motor controller already initialized");
        }
    }


    void move_forward(){
        //set motor to travel forward
        //set direction forward
        my_leftMotor.set_direction(true);
        my_rightMotor.set_direction(true);

        //enable the motor
        my_leftMotor.enable_motor();
        my_rightMotor.enable_motor();

    }


    //stop the motor
    void stop(){
        //disable the motor
        my_leftMotor.disable_motor();
        my_rightMotor.disable_motor();
    }

    void move_forward_dist(int distance_cm=10){
        if (initialized){
            //travel 10 cm

            //reset interrupt counters
            my_leftMotor.reset_interrupt_counter();
            my_rightMotor.reset_interrupt_counter();

            //set direction forward
            my_leftMotor.set_direction(true);
            my_rightMotor.set_direction(true);

            //enable the motor
            my_leftMotor.enable_motor();
            my_rightMotor.enable_motor();

            //wait while driving
            //move forward while distance traveld is less than 10 cm
            float left_travelled_dist_cm = (my_leftMotor.get_interrupt_counter_value()/PULSES_PER_WHEEL_ROTATION)*(2*M_PI*WHEEL_RADIUS);
            float right_travelled_dist_cm = (my_rightMotor.get_interrupt_counter_value()/PULSES_PER_WHEEL_ROTATION)*(2*M_PI*WHEEL_RADIUS);

            while (left_travelled_dist_cm < distance_cm-2){
                //wait 100 milliseconds
                //delay(100);

                //for testing print to see if they are the same
                /*
                Serial.print("Left distance (cm): ");
                Serial.println(left_travelled_dist_cm);
                Serial.print("Right distance (cm): ");
                Serial.println(right_travelled_dist_cm);
                Serial.println("");*/

                left_travelled_dist_cm = (my_leftMotor.get_interrupt_counter_value()/PULSES_PER_WHEEL_ROTATION)*(2*M_PI*WHEEL_RADIUS);
                right_travelled_dist_cm = (my_rightMotor.get_interrupt_counter_value()/PULSES_PER_WHEEL_ROTATION)*(2*M_PI*WHEEL_RADIUS);
            }

            left_travelled_dist_cm = (my_leftMotor.get_interrupt_counter_value()/PULSES_PER_WHEEL_ROTATION)*(2*M_PI*WHEEL_RADIUS);
            right_travelled_dist_cm = (my_rightMotor.get_interrupt_counter_value()/PULSES_PER_WHEEL_ROTATION)*(2*M_PI*WHEEL_RADIUS);

            Serial.print("Left distance (cm): ");
            Serial.println(left_travelled_dist_cm);
            Serial.print("Right distance (cm): ");
            Serial.println(right_travelled_dist_cm);
            Serial.println("");


            //disable the motor
            my_leftMotor.disable_motor();
            my_rightMotor.disable_motor();
        }else{
            Serial.println("Motor controller not initialized");
        }
    }


    //turn left
    void turn_90_degree_left(){
        if (initialized){
            //to turn left, left motor locked reverse, right motor forward, drive until reached
            Serial.println("Turning left");

            // Reset interrupt counters
            my_leftMotor.reset_interrupt_counter();
            my_rightMotor.reset_interrupt_counter();

            //the outer wheel needs to make 1/2 full rotation
            int needed_pulses = 40; //need to take into account that the motor does not stop instantly
            int interrupt_counter_val = my_rightMotor.get_interrupt_counter_value();

            // Set direction forward
            my_leftMotor.set_direction(false);
            my_rightMotor.set_direction(true);

            // Enable the motors
            //my_leftMotor.enable_motor();
            my_rightMotor.enable_motor();

            //lock the left motor
            my_leftMotor.lock_motor();

            //set the motors to max speed
            //my_leftMotor.update_PWMVal(255);
            my_rightMotor.update_PWMVal(255);


            //rotate until position is reached
            while (interrupt_counter_val < needed_pulses){
                interrupt_counter_val = my_rightMotor.get_interrupt_counter_value();
            }

            // Stop the motors
            my_leftMotor.disable_motor();
            my_rightMotor.disable_motor();

            Serial.println("Finished turning left");

        }
    }


    //turn right
    void turn_90_degree_right(){
        if (initialized){
            //to turn right, left motor forward, right motor locked reverse, drive until reached
            Serial.println("Turning right");

            // Reset interrupt counters
            my_leftMotor.reset_interrupt_counter();
            my_rightMotor.reset_interrupt_counter();

            //the outer wheel needs to make 1/2 full rotation
            int needed_pulses = 40; //need to take into account that the motor does not stop instantly
            int interrupt_counter_val = my_leftMotor.get_interrupt_counter_value();

            // Set direction forward
            my_leftMotor.set_direction(true);
            my_rightMotor.set_direction(false);

            // Enable the motors
            my_leftMotor.enable_motor();

            //lock the right motor
            my_rightMotor.lock_motor();

            //set the motors to max speed
            //my_leftMotor.update_PWMVal(255);
            my_leftMotor.update_PWMVal(255);


            //rotate until position is reached
            while (interrupt_counter_val < needed_pulses){
                interrupt_counter_val = my_leftMotor.get_interrupt_counter_value();
            }

            // Stop the motors
            my_leftMotor.disable_motor();
            my_rightMotor.disable_motor();

            Serial.println("Finished turning left");

        }
    }




    //testing cloosed loop motor control
    // Closed-loop control to make the motors spin at the same speed
    float move_forward_dist_closed_loop(float distance_cm) {
        if (initialized) {
            //define all the variables
            // Initial distances
            float left_travelled_dist_cm = 0;
            float right_travelled_dist_cm = 0;

            //perform the speed adjustment only every 5 counts (50ms)/the distance check should be performed more frequently
            int count = 0;
            float speed_difference;
            int left_PWM;
            int right_PWM;
            float stop_dist_cm = distance_cm; //- 1.0;

            // Reset interrupt counters
            my_leftMotor.reset_interrupt_counter();
            my_rightMotor.reset_interrupt_counter();
            //start the motor
            move_forward();

            //move while the stop distance has not been reached
            while (left_travelled_dist_cm < stop_dist_cm) {
                // Wait for a short period of time, do this before measuring to get the most recent value
                delay(10);

                // read distance of left motor
                left_travelled_dist_cm = (float(my_leftMotor.get_interrupt_counter_value()) / PULSES_PER_WHEEL_ROTATION) * WHEEL_CIRCOMFERENCE_CM;

                if (count >= 5) {
                    //Reset the counter
                    count = 0;
                    //read distance of right motor
                    right_travelled_dist_cm =  (float(my_rightMotor.get_interrupt_counter_value()) / PULSES_PER_WHEEL_ROTATION) * WHEEL_CIRCOMFERENCE_CM;

                    // Calculate speed difference
                    speed_difference = (right_travelled_dist_cm - left_travelled_dist_cm) * KP;

                    //Adjust the motor speeds
                    left_PWM = my_leftMotor.get_current_PWMVal() + speed_difference;
                    right_PWM = my_rightMotor.get_current_PWMVal() - speed_difference;
                    my_leftMotor.update_PWMVal(left_PWM);
                    my_rightMotor.update_PWMVal(right_PWM);
                }
                count++;
            }

            // Stop the motors
            stop();

            //calculate distance travelled
            left_travelled_dist_cm = (float(my_leftMotor.get_interrupt_counter_value()) / PULSES_PER_WHEEL_ROTATION) * WHEEL_CIRCOMFERENCE_CM;
            right_travelled_dist_cm = (float(my_leftMotor.get_interrupt_counter_value()) / PULSES_PER_WHEEL_ROTATION) * WHEEL_CIRCOMFERENCE_CM;
            /*
            Serial.print("Left distance travelled (cm): ");
            Serial.println(left_travelled_dist_cm);
            Serial.print("Right distance travelled (cm): ");
            Serial.println(right_travelled_dist_cm);*/
            return (left_travelled_dist_cm + right_travelled_dist_cm)/2;
        } else {
            Serial.println("Motor controller not initialized");
            return -1;
        }
    }



    void test_right_motor(){
        if (initialized){
            //test the right motor
            Serial.println("Test right motor");

            my_rightMotor.reset_interrupt_counter();
            //set direction forward
            my_rightMotor.set_direction(true);
            //enable the motor
            my_rightMotor.enable_motor();

            delay(1000);

            unsigned long counter_val = my_rightMotor.get_interrupt_counter_value();

            //disable the motor
            my_rightMotor.disable_motor();

            Serial.print("Counter value: ");
            Serial.println(counter_val);
        }else{
            Serial.println("Motor controller not initialized");
        }
    }

    void test_left_motor(){
        if (initialized){
            //test the right motor
            Serial.println("Test left motor");

            my_leftMotor.reset_interrupt_counter();
            //set direction forward
            my_leftMotor.set_direction(true);
            //enable the motor
            my_leftMotor.enable_motor();

            delay(1000);

            unsigned long counter_val = my_leftMotor.get_interrupt_counter_value();

            //disable the motor
            my_leftMotor.disable_motor();

            Serial.print("Counter value: ");
            Serial.println(counter_val);
        }else{
            Serial.println("Motor controller not initialized");
        }
    }
};



#endif //LIBRARIES_MOTORCONTROL_H
