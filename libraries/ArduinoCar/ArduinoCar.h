
#ifndef LIBRARIES_ARDUINOCAR_H
#define LIBRARIES_ARDUINOCAR_H

#include <Arduino.h>
#include <MotorControl.h>
#include <ArduinoCarSensors.h>



//create the arduino Car that controls everything

class ArduinoCar{
private:
    //create the objects to move and get sensor values
    MotorController my_motorController;
    CustomServo my_servo;
    ProximitySensor my_proximitySensor;

    boolean initialized;
    boolean reached_goal;

    //keep track of the distance travelled to determine position
    /*float travelled_x_dist;
    float travelled_y_dist;
    int estimated_rotation;*/

public:
    ArduinoCar(){
        initialized = false;
        reached_goal = false;

        //initialize the distance to 0
        /*
        travelled_x_dist = 0;
        travelled_y_dist = 0;
        estimated_rotation = 0; //start position has 0 degree */

    }

    void setup_arduino_car(int ser_pin, int trig_pin, int ech_pin,
                           int l_ena_pin, int l_dir_pin, int l_feed_pin,
                           int r_ena_pin, int r_dir_pin, int r_feed_pin){
        if (!initialized){
            //setup each component of the arduino car
            //setup the servo (270 degree servo)
            my_servo.setup_servo(ser_pin, 270);
            //setup the proximity sensor
            my_proximitySensor.setup_proximity_sensor(trig_pin, ech_pin);
            //setup the motor controller
            my_motorController.setup_motor_controller(l_ena_pin,
                                                      l_dir_pin,
                                                      l_feed_pin,
                                                      r_ena_pin,
                                                      r_dir_pin,
                                                      r_feed_pin);

            initialized = true;

            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(95);
            delay(100);
        }else{
            Serial.println("ArduinoCar already initialized");
        }
    }

    void run(){
        if (initialized){
            Serial.println("running car");

            //initial position (at start)
            float travelled_x_dist = 0.0;
            float travelled_y_dist = 0.0;
            int rotation = 0;

            float max_wall_dist = 35.0; //adjust value

            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(95);
            delay(1000);

            //the current state
            //get sensor reading
            int right_dist, left_dist, front_dist;
            measure_distances(right_dist, front_dist, left_dist);
            Serial.println("Distance: ");
            Serial.print("Right dist (cm): ");
            Serial.println(right_dist);
            Serial.print("Front dist (cm): ");
            Serial.println(front_dist);
            Serial.print("Left dist (cm): ");
            Serial.println(left_dist);

            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(95);
            delay(1000);

            //get obstacles (using the distances
            boolean is_wall_left = left_dist < max_wall_dist; //if the left distance is smaller than the maximum wall distance
            boolean is_wall_right = right_dist < max_wall_dist;
            boolean is_wall_front = front_dist < max_wall_dist;
            boolean is_wall_back = true; //set like this initally (after that it will always false as it has to come from that state)

            int action; //no action taken so far

            //run main body until terminate action (10) is sent
            while (true){
                //ask agent for action (using distance travelled)
                action = get_action_from_agent(travelled_x_dist, travelled_y_dist, rotation);

                if (action == 10){
                    //if action is 10 (end program)
                    break;
                }

                //Note: action (which way to move) depends on rotation
                //solution:  for arduino: 0: go straight, 1: go back, 2: right, 3: left, 10: terminate
                //check if action cannot be taken skip iteration ->  (continue; )
                boolean perform_action = true;
                switch(action){
                    case 0:
                        //go straight
                        if (is_wall_front){
                            //do no action
                            perform_action = false;
                        }
                        break;
                    case 1:
                        //go back
                        if (is_wall_back){
                            //do no action
                            perform_action = false;
                        }
                        break;
                    case 2:
                        //go right
                        if (is_wall_right){
                            //do no action
                            perform_action = false;
                        }
                        break;
                    case 3:
                        //go left
                        if (is_wall_left){
                            //do no action
                            perform_action = false;
                        }
                        break;
                }

                //take action + update position
                if (perform_action){
                    Serial.println("Performing action");
                }else{
                    Serial.println("Not performing action");
                }


                //get sensor readings
                measure_distances(right_dist, front_dist, left_dist);
                Serial.println("Distance: ");
                Serial.print("Right dist (cm): ");
                Serial.println(right_dist);
                Serial.print("Front dist (cm): ");
                Serial.println(front_dist);
                Serial.print("Left dist (cm): ");
                Serial.println(left_dist);

                //update obstacles readings
                is_wall_left = left_dist < max_wall_dist; //if the left distance is smaller than the maximum wall distance
                is_wall_right = right_dist < max_wall_dist;
                is_wall_front = front_dist < max_wall_dist;
                is_wall_back = false; // always false as it has to come from that state

                //set the servo to 90° (a bit more to be 90° physically)
                my_servo.move_to(95);
                delay(1000);
            }

            Serial.println("At goal");

            /*
             //get sensor reading
            int right_dist, left_dist, front_dist;
            measure_distances(right_dist, front_dist, left_dist);
            Serial.println("Distance: ");
            Serial.print("Right dist (cm): ");
            Serial.println(right_dist);
            Serial.print("Front dist (cm): ");
            Serial.println(front_dist);
            Serial.print("Left dist (cm): ");
            Serial.println(left_dist);

            //find obstacles

            //calculate current position (state)
            float travelled_x_dist = 0.0;
            float travelled_y_dist = 0.0;
            int rotation = 0;

            //ask agent for action
            int action = get_action_from_agent(travelled_x_dist, travelled_y_dist);

            //perform action (test)
            if (action == 0){
                travelled_x_dist = 10;
            }else if (action == 1){
                travelled_x_dist = 20;
            }else if (action == 2){
                travelled_x_dist = 30;
            }else{
                //action is 3
                travelled_x_dist = 10;
            }

            //ask agent for action
            action = get_action_from_agent(travelled_x_dist, travelled_y_dist);
            */
            /**
             *- Read the distance around the car
                - Set servo to 0 degree
                - Read distance
                - Set servo to 90 degree
                - Read distance
                - Set servo to 180 degree
                - read distance
            - Calculate if obstacle or not for each direction (if distance is x or smaller)
            - Calculate current state (the position)
            - Ask agent for action
            - Perform action (if the action would result in a crash, dont do anything)
             */

            /*
            maze_test();*/


            //run the main loop while we have not reached the goal
            //while (!reached_goal){
            //get sensor reading
            //send senros reading to laptop
            //get action from laptor (left, right, front back) (which block to move to)
            //chek if action possible (if obstacle in the way, skip to end
            //perform action (move to the next state)
            //}

            //testing control
            //drive forward until obstacle is 20cm away, then stop



            /*
            //read distance
            int dist_cm;
            my_proximitySensor.measure_distance(dist_cm);

            //enable motor moving forward
            my_motorController.move_forward();

            while (dist_cm > 20){
                //run motor
                Serial.print("Distance: ");
                Serial.print(dist_cm);
                Serial.println(" cm");
                //update distance reading
                my_proximitySensor.measure_distance(dist_cm);
            }

            //stop the motor
            my_motorController.stop();
            Serial.print("Distance: ");
            Serial.print(dist_cm);
            Serial.println(" cm");

            Serial.println("Finished moving");
             */

            /*measure_distances(int& dist_0_degree, int& dist_90_degree, int& dist_180_degree)*/
            //measure the distance to all 3 sides
            /*int right_dist, left_dist, front_dist;
            measure_distances(right_dist, front_dist, left_dist);
            Serial.println("Distance: ");
            Serial.print("Right dist (cm): ");
            Serial.println(right_dist);
            Serial.print("Front dist (cm): ");
            Serial.println(front_dist);
            Serial.print("Left dist (cm): ");
            Serial.println(left_dist);*/


        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    void maze_test(){
        //driving a preprogrammed path in the maze for testing of senosor and motor contorl
        if (initialized){
            Serial.println("Maze test");
            //navigation
            //go straing until (45cm in front of the wall)(travel one block at a time)
            // turn left
            //measure distance front (go straing until 5cm in front of obstacle)
            //turn right
            //go measure distance front (go straing unitl 5cm in front of obstacle)
            //turn right
            //measrue distance fron (go straing unitl 5cm in front of obstacle)
            //at goal

            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(95);
            delay(1000);

            //travel straight 3 blocks
            float actual_dist;
            for (int i=0; i < 3; i++){
                actual_dist = my_motorController.move_forward_dist_closed_loop(40);
                delay(1000);
            }


            //read distance
            int dist_cm;
            my_proximitySensor.measure_distance(dist_cm);

            //enable motor moving forward
            my_motorController.move_forward();

            while (dist_cm > 45){
                Serial.print("Distance: ");
                Serial.print(dist_cm);
                Serial.println(" cm");
                //move one block straigth
                my_proximitySensor.measure_distance(dist_cm);
            }

            //stop the motor
            my_motorController.stop();
            Serial.print("Distance: ");
            Serial.print(dist_cm);
            Serial.println(" cm");

            //turn left
            my_motorController.turn_90_degree_left();
            delay(1000);

            //get distance to wall
            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(95);
            delay(1000);
            my_proximitySensor.measure_distance(dist_cm);
            //travel the distance
            float dist_to_travel = float(dist_cm) - 5.0;
            actual_dist = my_motorController.move_forward_dist_closed_loop(120);


            Serial.println("Finished moving");

        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    void measure_distances(int& dist_0_degree, int& dist_90_degree, int& dist_180_degree){
        if (initialized){
            //move servo to 0 degree
            my_servo.move_to(0);
            delay(2000);
            //measure the distance at 0 degree
            my_proximitySensor.measure_distance(dist_0_degree);

            //move servo to 90 degree
            my_servo.move_to(95);
            delay(2000);
            //measure the distance at 90 degree
            my_proximitySensor.measure_distance(dist_90_degree);

            //move servo to 180 degree
            my_servo.move_to(200);
            delay(2000);
            //measure the distance at 90 degree
            my_proximitySensor.measure_distance(dist_180_degree);
        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    void test_components(){
        if (initialized){
            Serial.println("Testing each component individually");

            //test servo (move from 0, to 90 to 180)
            Serial.println("Servo 0");
            my_servo.move_to(0);
            delay(2000);
            Serial.println("Servo 90");
            my_servo.move_to(95);
            delay(2000);
            Serial.println("Servo 180");
            my_servo.move_to(200);
            delay(2000);

            //move to front
            Serial.println("Servo 90");
            my_servo.move_to(95);
            delay(2000);

            //test proximity sensor
            int dist_cm;
            my_proximitySensor.measure_distance(dist_cm);
            Serial.print("Distance: ");
            Serial.print(dist_cm);
            Serial.println(" cm");

            //1 second wait
            delay(1000);

            //travel forward 10cm
            //Serial.println("Travelling 10cm forward");
            //my_motorController.move_forward_dist(10);

            my_motorController.test_right_motor();
            my_motorController.test_left_motor();

            Serial.println("Finished test");
        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    int get_action_from_agent(float x_pos_cm, float y_pos_cm, int rot){
        if (initialized){
            //send position as: "Position,<x_cm_pos>,<y_cm_pos>"
            Serial.print("Position,");
            Serial.print(x_pos_cm);
            Serial.print(",");
            Serial.print(y_pos_cm);
            Serial.print(",");
            Serial.print(rot);
            Serial.println();

            //get action from agent
            //wait for serial
            while (!Serial.available());

            int action = Serial.readString().toInt();
            return action;
        }
    }
};



#endif //LIBRARIES_ARDUINOCAR_H
