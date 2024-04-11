
#ifndef LIBRARIES_ARDUINOCAR_H
#define LIBRARIES_ARDUINOCAR_H

#include <Arduino.h>
#include <MotorControl.h>
#include <ArduinoCarSensors.h>


//ArduinoCar class that controls all components
class ArduinoCar{
private:
    //create the objects to move and get sensor values
    MotorController my_motorController;
    CustomServo my_servo;
    ProximitySensor my_proximitySensor;

    bool initialized;
    bool reached_goal;

    //keep track of the distance travelled to determine position
    float travelled_x_dist;
    float travelled_y_dist;
    int rotation;
    bool is_travelling_backwards; //keep track of which direction the car is travelling

    //keep track of walls
    bool is_wall_left;
    bool is_wall_right;
    bool is_wall_front;
    bool is_wall_back;
    int current_left_dist;
    int current_right_dist;

    //keep track of the current state
    struct State{
        int row = 0;
        int col = 0;
    };
    State current_state;


    //state observation struct to store the information for each visited state
    struct stateObservation{
        bool is_wall_left = true;
        bool is_wall_right = true;
        bool is_wall_front = true;
        bool is_wall_back = true;
        bool is_initialized = false;
        int rotation = 0;
    };

    //state observations matrix
    static const int num_maze_rows = 3; //number of y steps
    static const int num_maze_cols = 6; //number of x steps
    stateObservation mazeObservations[num_maze_rows][num_maze_cols];

    //constants
    const float block_size = 40;
    const float max_wall_dist = 35.0; //adjust value
    const float forward_step_travel_dist_cm = 40; //20; //size of one step (i think 39 to 40 cm, measure)
    const float turn_travel_dist_reduction_cm = 3; //measure (center point move backwards as it turns around the axis at the wheels and not the center)
    const float after_turn_step_travel_dist_cm = 30; //measure

    const int servo_0_deg_val = 0;
    const int servo_90_deg_val = 90; //needs to be adjusted depending on servo, sometimes it needs to be bit higher than the wanted value
    const int servo_180_deg_val = 180; // -||-

    //private function not intended to be used outside the class
    void reset(){
        //reset all the variables
        travelled_x_dist = 0.0;
        travelled_y_dist = 0.0;
        rotation = 0;
        is_travelling_backwards = false;

        //reset the maze observation matrix
        for (int i = 0; i < num_maze_rows; i++){
            for (int j = 0; j < num_maze_cols; j++){
                mazeObservations[i][j].is_initialized = false;
            }
        }

        //reset the state
        current_state.row = 0;
        current_state.col = 0;

        //set the initial wall observations
        is_wall_left = true;
        is_wall_right = true;
        is_wall_front = false;
        is_wall_back = true;
        current_left_dist = -1;
        current_right_dist = -1;

        //update the current state matrix
        mazeObservations[current_state.row][current_state.col].is_wall_left = is_wall_left;
        mazeObservations[current_state.row][current_state.col].is_wall_right = is_wall_right;
        mazeObservations[current_state.row][current_state.col].is_wall_front = is_wall_front;
        mazeObservations[current_state.row][current_state.col].is_wall_back = is_wall_back;
        mazeObservations[current_state.row][current_state.col].is_initialized = true;
        mazeObservations[current_state.row][current_state.col].rotation = rotation;
    }

    int get_action_from_agent(int x_pos, int y_pos, int rot){
        if (initialized){
            //send position as: "Position,<x_cm_pos>,<y_cm_pos>"
            Serial.print("Position,");
            Serial.print(x_pos);
            Serial.print(",");
            Serial.print(y_pos);
            Serial.print(",");
            Serial.print(rot);

            //sending wall information
            Serial.print(",");
            Serial.print(is_wall_left);
            Serial.print(",");
            Serial.print(is_wall_right);
            Serial.print(",");
            Serial.print(is_wall_front);
            Serial.print(",");
            Serial.print(is_wall_back);
            //end wall information

            //end of line
            Serial.println();

            //get action from agent
            //wait for serial
            while (!Serial.available());

            int action = Serial.readString().toInt();
            return action;
        }
    }

    void update_state(){
        //update the current state struct using the distance travelled
        current_state.row = static_cast<int>(travelled_y_dist / block_size + 0.5);
        current_state.col = static_cast<int>(travelled_x_dist / block_size + 0.5);
    }

    void go_straight(){
        Serial.println("Going straight");
        //measure the distance to the front wall to avoid running into it
        int front_dist;
        measure_front_distance(front_dist);
        //adjust for space needed at the front
        front_dist = front_dist - 7;
        //select the distance to travel forward
        float travel_dist = min(forward_step_travel_dist_cm, front_dist);

        //travel forward
        //mode == 1: //left motor needs to go faster
        //mode == 2: right motor needs to go faster
        if (current_left_dist < 15 & current_left_dist != -1){
            Serial.println("mode 1");
            //increase left motor speed -> mode = 1
            my_motorController.move_forward_dist_closed_loop(travel_dist, 1);
        }else if (current_right_dist < 15 & current_right_dist != -1){
            //increase right motor speed -> mode = 2
            Serial.println("mode 2");
            my_motorController.move_forward_dist_closed_loop(travel_dist, 2);
        }else {
            //do the normal control -> mode = 0 (default)
            my_motorController.move_forward_dist_closed_loop(travel_dist);
        }

        //update position, rotation is unchanged
        switch(rotation){
            case 0:
                //x = x + dist
                travelled_x_dist = travelled_x_dist + forward_step_travel_dist_cm;
                break;
            case 90:
                //y = y - dist
                travelled_y_dist = travelled_y_dist - forward_step_travel_dist_cm;
                break;
            case 180:
                //x = x -dist
                travelled_x_dist = travelled_y_dist - forward_step_travel_dist_cm;
                break;
            case 270:
                //y = y + dist
                travelled_y_dist  = travelled_y_dist + forward_step_travel_dist_cm;
        }
    }

    void go_back(){
        Serial.println("Going backwards");

        //update direction of travel (so observation can be taken from saved observations)
        is_travelling_backwards = true;
        my_motorController.move_backward_dist_closed_loop(forward_step_travel_dist_cm); //for testing, adjust distance

        //update position, rotation is unchanged
        switch(rotation){
            case 0:
                //x = x - dist
                travelled_x_dist = travelled_x_dist - forward_step_travel_dist_cm;
                break;
            case 90:
                //y = y + dist
                travelled_y_dist = travelled_y_dist + forward_step_travel_dist_cm;
                break;
            case 180:
                //x = x + dist
                travelled_x_dist = travelled_y_dist + forward_step_travel_dist_cm;
                break;
            case 270:
                //y = y - dist
                travelled_y_dist  = travelled_y_dist - forward_step_travel_dist_cm;
        }
    }


    void go_right(){
        Serial.println("Turning right");

        //measure distance at the front of the car to avoid running in to the wall
        int front_dist;
        measure_front_distance(front_dist);
        //adjust for space needed at the front
        front_dist = front_dist - 5;
        //select the distance to travel forward
        float travel_dist = min(turn_travel_dist_reduction_cm, front_dist);
        //travel forward the distance that will be lost during turning
        my_motorController.move_forward_dist_closed_loop(travel_dist);
        delay(1000);

        //perform right turn
        my_motorController.turn_90_degree_right();
        delay(1000);
        //update rotation (rot = rot -90)
        rotation = ((rotation - 90)%360 + 360)%360; //needs to handle negative numbers

        //travel forward to reach the center of the new square
        //measure distance at the front of the car to avoid running in to the wall
        measure_front_distance(front_dist);
        //adjust for space needed at the front (7cm)
        front_dist = front_dist - 7;
        //select the distance to travel forward
        travel_dist = min(after_turn_step_travel_dist_cm, front_dist);
        //travel forward the distance that will be lost during turning
        my_motorController.move_forward_dist_closed_loop(travel_dist);

        //update position
        switch(rotation){
            case 0:
                //x = x + dist
                travelled_x_dist = travelled_x_dist + forward_step_travel_dist_cm;
                break;
            case 90:
                //y = y - dist
                travelled_y_dist = travelled_y_dist - forward_step_travel_dist_cm;
                break;
            case 180:
                //x = x -dist
                travelled_x_dist = travelled_y_dist - forward_step_travel_dist_cm;
                break;
            case 270:
                //y = y + dist
                travelled_y_dist  = travelled_y_dist + forward_step_travel_dist_cm;
        }
    }


    void go_left(){
        Serial.println("Turning left");

        //measure distance at the front of the car to avoid running in to the wall
        int front_dist;
        measure_front_distance(front_dist);
        //adjust for space needed at the front
        front_dist = front_dist - 5;
        //select the distance to travel forward
        float travel_dist = min(turn_travel_dist_reduction_cm, front_dist);
        //travel forward the distance that will be lost during turning
        my_motorController.move_forward_dist_closed_loop(travel_dist);
        delay(1000);

        //perform left turn
        my_motorController.turn_90_degree_left();
        delay(1000);
        //update rotation (rot = rot + 90)
        rotation = ((rotation + 90)%360 + 360)%360; //needs to handle negative numbers

        //travel forward to reach the center of the new square
        //measure distance at the front of the car to avoid running in to the wall
        measure_front_distance(front_dist);
        //adjust for space needed at the front (7cm)
        front_dist = front_dist - 7;
        //select the distance to travel forward
        travel_dist = min(after_turn_step_travel_dist_cm, front_dist);
        //travel forward the distance that will be lost during turning
        my_motorController.move_forward_dist_closed_loop(travel_dist);

        //update position
        switch(rotation){
            case 0:
                //x = x + dist
                travelled_x_dist = travelled_x_dist + forward_step_travel_dist_cm;
                break;
            case 90:
                //y = y - dist
                travelled_y_dist = travelled_y_dist - forward_step_travel_dist_cm;
                break;
            case 180:
                //x = x -dist
                travelled_x_dist = travelled_y_dist - forward_step_travel_dist_cm;
                break;
            case 270:
                //y = y + dist
                travelled_y_dist  = travelled_y_dist + forward_step_travel_dist_cm;
        }
    }


    void update_observation(){
        //if going backwards (get from state matrix, else get reading)
        if (is_travelling_backwards){
            is_travelling_backwards = false;
            Serial.println("Getting observations from matrix");
            if (mazeObservations[current_state.row][current_state.col].is_initialized){
                //depending on current rotation and the rotation at the time of taking the measurement update the is wall variables
                switch (rotation) {
                    case 0:
                        //depending on rotation when taking the measurement
                        switch (mazeObservations[current_state.row][current_state.col].rotation) {
                            case 0:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_right= mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                break;
                            case 90:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                break;
                            case 180:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                break;
                            case 270:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                break;
                        }
                        break;
                    case 90:
                        //depending on rotation when the measurement is taken
                        switch (mazeObservations[current_state.row][current_state.col].rotation){
                            case 0:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                break;
                            case 90:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                break;
                            case 180:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                break;
                            case 270:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_front;
                        }
                        break;
                    case 180:
                        //update wall locations
                        switch (mazeObservations[current_state.row][current_state.col].rotation){
                            case 0:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                break;
                            case 90:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                break;
                            case 180:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                break;
                            case 270:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_left;
                        }
                        break;
                    case 270:
                        //update wall locations
                        switch (mazeObservations[current_state.row][current_state.col].rotation){
                            case 0:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                break;
                            case 90:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                break;
                            case 180:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_back;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                break;
                            case 270:
                                is_wall_left = mazeObservations[current_state.row][current_state.col].is_wall_left;
                                is_wall_right = mazeObservations[current_state.row][current_state.col].is_wall_right;
                                is_wall_front = mazeObservations[current_state.row][current_state.col].is_wall_front;
                                is_wall_back = mazeObservations[current_state.row][current_state.col].is_wall_back;
                        }
                        break;
                }

            }else{
                Serial.println("Error, could not get observations from state matrix");
                //set all directions to wall to avoid crashing
                is_wall_left = true;
                is_wall_right = true;
                is_wall_front = true;
                is_wall_back = true;
            }
        }else{
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
            my_servo.move_to(servo_90_deg_val);
            delay(500);

            //get obstacles (using the distances
            is_wall_left = left_dist < max_wall_dist; //if the left distance is smaller than the maximum wall distance
            is_wall_right = right_dist < max_wall_dist;
            is_wall_front = front_dist < max_wall_dist;
            is_wall_back = false; // always false as it has to come from that state

            current_left_dist = left_dist;
            current_right_dist = right_dist;

            //update the state observation matrix
            mazeObservations[current_state.row][current_state.col].is_wall_left = is_wall_left;
            mazeObservations[current_state.row][current_state.col].is_wall_right = is_wall_right;
            mazeObservations[current_state.row][current_state.col].is_wall_front = is_wall_front;
            mazeObservations[current_state.row][current_state.col].is_wall_back = is_wall_back;
            mazeObservations[current_state.row][current_state.col].is_initialized = true;
            mazeObservations[current_state.row][current_state.col].rotation = rotation; //store the rotation at which the measurement was made
        }
    }


public:
    ArduinoCar(){
        initialized = false;
        reached_goal = false;

        //initialize the distance to 0
        travelled_x_dist = 0;
        travelled_y_dist = 0;
        rotation = 0; //start position has 0 degree
        is_travelling_backwards = false;

        //set the initial wall observations
        is_wall_left = true;
        is_wall_right = true;
        is_wall_front = false;
        is_wall_back = true;
        current_left_dist = -1;
        current_right_dist = -1;
    }

    void setup_arduino_car(int ser_pin, int trig_pin, int ech_pin,
                           int l_ena_pin, int l_dir_pin, int l_feed_pin,
                           int r_ena_pin, int r_dir_pin, int r_feed_pin){
        if (!initialized){
            //setup each component of the arduino car
            //setup the servo (270 degree servo)
            my_servo.setup_servo(ser_pin, 180);
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
            my_servo.move_to(servo_90_deg_val);
            delay(100);
        }else{
            Serial.println("ArduinoCar already initialized");
        }
    }



    void run(){
        if (initialized){
            Serial.println("running car");

            //reset all variables
            reset();

            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(servo_90_deg_val);
            delay(1000);

            int action; //no action taken so far

            //run main body until terminate action (10) is sent
            while (true){
                //print observations (for testing)
                Serial.println("Observations: ");
                Serial.print("is_wall_left: ");
                Serial.println(is_wall_left);
                Serial.print("is_wall_right: ");
                Serial.println(is_wall_right);
                Serial.print("is_wall_front: ");
                Serial.println(is_wall_front);
                Serial.print("is_wall_back: ");
                Serial.println(is_wall_back);
                Serial.print("rotation: ");
                Serial.println(rotation);

                //ask agent for action (using the current state and rotation)
                action = get_action_from_agent(current_state.row, current_state.col, rotation);

                if (action == 10){
                    //if action is 10 (end program)
                    break;
                }

                //check if action cannot be taken skip iteration
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
                    //do the movement that corresponds to the action
                    switch(action){
                        case 0:
                            //go straight
                            go_straight();
                            break;
                        case 1:
                            //go back
                            go_back();
                            break;
                        case 2:
                            //go right
                            go_right();
                            break;
                        case 3:
                            //go left
                            go_left();
                            break;
                    }

                    delay(500);

                    //update the state after taken the action
                    update_state();

                    //only get the sensor readings when an action was performed, no action -> no state change
                    //update sensor readings after performing action and updating the state
                    update_observation();
                }else{
                    Serial.println("Not performing action");
                }
            }
            Serial.println("At goal");
        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    void maze_test(){
        //driving a pre-programmed path in the maze for testing of sensor and motor control
        if (initialized){
            Serial.println("Maze test");
            //navigation
            //go straight until (45cm in front of the wall)(travel one block at a time)
            // turn left
            //measure distance front (go straight until 5cm in front of obstacle)
            //turn right
            //go measure distance front (go straight unitl 5cm in front of obstacle)
            //turn right
            //measrue distance fron (go straight unitl 5cm in front of obstacle)
            //at goal

            //set the servo to 90° (a bit more to be 90° physically)
            my_servo.move_to(servo_90_deg_val);
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
            my_servo.move_to(servo_90_deg_val);
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
            my_servo.move_to(servo_0_deg_val);
            delay(2000);
            //measure the distance at 0 degree
            my_proximitySensor.measure_distance(dist_0_degree);

            //move servo to 90 degree
            my_servo.move_to(servo_90_deg_val);
            delay(2000);
            //measure the distance at 90 degree
            my_proximitySensor.measure_distance(dist_90_degree);

            //move servo to 180 degree
            my_servo.move_to(servo_180_deg_val);
            delay(2000);
            //measure the distance at 90 degree
            my_proximitySensor.measure_distance(dist_180_degree);
        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    void measure_front_distance(int& dist){
        if (initialized){
            //measure the distance to the wall in front of the car
            my_proximitySensor.measure_distance(dist);
        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }


    void test_components(){
        if (initialized){
            Serial.println("Testing each component individually");

            //test servo (move from 0, to 90 to 180)
            Serial.println("Servo 0");
            my_servo.move_to(servo_0_deg_val);
            delay(2000);
            Serial.println("Servo 90");
            my_servo.move_to(servo_90_deg_val);
            delay(2000);
            Serial.println("Servo 180");
            my_servo.move_to(servo_180_deg_val);
            delay(2000);

            //move to front
            Serial.println("Servo 90");
            my_servo.move_to(servo_90_deg_val);
            delay(2000);

            //test proximity sensor
            int dist_cm;
            my_proximitySensor.measure_distance(dist_cm);
            Serial.print("Distance: ");
            Serial.print(dist_cm);
            Serial.println(" cm");

            //1 second wait
            delay(1000);

            //test motors
            my_motorController.test_right_motor();
            my_motorController.test_left_motor();

            Serial.println("Finished test");
        }else{
            Serial.println("ArduinoCar not initialized");
        }
    }
};



#endif //LIBRARIES_ARDUINOCAR_H
