import serial
import time


#base car control 
class ArduinoCarControl: 
    #arduino car:  [0: go straight, 1: go back, 2: right, 3: left, 10: terminate] relative to current position and rotation
    #agent : [0: up, 1: down, 2: right, 3: left, 10: terminate]
    
    #mapping of the agents actions to the cars actions
    agent_car_action_mapping = {
        #rot = 0: agent down-> right, agent up -> left, agent left -> back, agent right -> forward
        0: {1: 2, 0: 3, 3: 1, 2: 0, 10: 10}, 
        #rot = 90: agent down-> back, agent up -> forward, agent left -> left, agent right -> right
        90: {1: 1, 0: 0, 3: 3, 2: 2, 10: 10}, 
        #rot = 180: agent down-> left, agent up -> right, agent left -> forward, agent right -> back
        180: {1: 3, 0: 2, 3: 0, 2: 1, 10: 10}, 
        #rot = 270: agent down-> forward, agent up -> back, agent left -> right, agent right -> left
        270: {1: 0, 0: 1, 3: 2, 2: 3, 10: 10}, 
    } 
    

    def __init__(self) -> None:
        #open serial connection to communicate with the arduino
        self.arduino = serial.Serial(port='COM3', baudrate=115200, timeout=0.1)
 
    def read_position(self): 
        #// Send postion as: "Position,<x_cm_pos>,<y_cm_pos>"
        while True: 
            data = self.arduino.readline().decode('utf-8').strip()
            #check if something was sent 
            if data: 
                split_data =data.split(",")
                #check if keyword present 
                if split_data[0] == "Position": 
                    #return the estimated position 
                    return int(split_data[1]), int(split_data[2]), int(split_data[3])
                else: 
                    print(data)
            else: 
                #wait for a short period of time 
                time.sleep(0.05)

    def write_action(self, action): 
        self.arduino.write(bytes(str(action), 'utf-8'))

    def run(self):
        pass 

#class that allows for control from the terminal 
class ArduinoCarControlManual(ArduinoCarControl): 
    def __init__(self) -> None:
        super().__init__()

    #overwrite the run method 
    def run(self) -> None:
        action = None; 
        while action != 101:
            row, col, rotation = self.read_position()
            print(f"Position: {row}, {col}, rotation: {rotation} degree")
            possible_actions = [0, 1, 2, 3, 10]
            while True: 
                action = int(input("Enter an action [for arduino: 0: go straight, 1: go back, 2: right, 3: left, 10: terminate]: "))
                if action in possible_actions: 
                    break
            self.write_action(action) 


if __name__ == "__main__":
    print("Start")
    #manual control setup
    ArduinoCar = ArduinoCarControlManual()
    