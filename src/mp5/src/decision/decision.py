import pickle
import math
import numpy as np
class VehicleDecision():
    def __init__(self, fn):
        self.waypoint_list = pickle.load(open(fn,'rb')) # a list of waypoints
        self.pos_idx = int(1)
        self.vehicle_state = 'middle'
        self.counter = 0
        self.previousPerception = 0
        self.distChangeCount = 0 
        
    def get_ref_state(self, currState, perceptionInput,pedPosition,distance):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs: 
                currState: ModelState, the current state of vehicle
                perceptionInput: float, currently the distance between vehicle and obstacle in front
            Outputs: reference state position and velocity of the vehicle 
        """
        # positive value means car is moving closer to pedestrian
        differencePosition = self.previousPerception - perceptionInput 
        
        
        # print("Current Distance:",perceptionInput)
        # print("Current Distance:",distance)
        
        if not math.isnan(differencePosition) and differencePosition!=0:
            print("Dist Change",self.distChangeCount)
            print("Previous Distance:",self.previousPerception)    
            print("Current Distance:",perceptionInput)
            print("Difference:",differencePosition)
            self.distChangeCount = 0
        
        self.distChangeCount += 1
        self.previousPerception = perceptionInput
        curr_x = currState.pose.position.x
        curr_y = currState.pose.position.y
        front_dist = perceptionInput
        # print(type(pedPosition))
        # difx = pedPosition[0] - curr_x
        # dify = (curr_y-pedPosition[1])**2
        # dis = np.sqrt( + )
        # print("Ped Position:",pedPosition)
        # print("Perception Input:",perceptionInput)
        # print("Distance",distance)
        

        # If the distance between vehicle and obstacle in front is less than 15, stop the vehicle
        if front_dist < 15:
            target_x = curr_x
            target_y = curr_y
            ref_v = -1
        else:
            target_x = self.waypoint_list[self.pos_idx][0]
            target_y = self.waypoint_list[self.pos_idx][1]

            curr_x = currState.pose.position.x
            curr_y = currState.pose.position.y
            
            distToTargetX = abs(target_x - curr_x)
            distToTargetY = abs(target_y - curr_y)

            if ((distToTargetX < 0.5 and distToTargetY < 0.5)) or self.counter > 100:
                self.counter = 0
                self.pos_idx += 1
                self.pos_idx = int(self.pos_idx % len(self.waypoint_list))
                print("reached",self.waypoint_list[self.pos_idx-1][0],self.waypoint_list[self.pos_idx-1][1],
                    "next",self.waypoint_list[self.pos_idx][0],self.waypoint_list[self.pos_idx][1])
            else:
                self.counter += 1
            ref_v = 5

        return [target_x, target_y, ref_v]
