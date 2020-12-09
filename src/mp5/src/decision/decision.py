import pickle
import math
from ackermann_msgs.msg import AckermannDrive
import numpy as np
import rospy

class VehicleDecision():
    def __init__(self, fn):
        self.waypoint_list = pickle.load(open(fn,'rb')) # a list of waypoints
        self.pos_idx = int(1)
        self.vehicle_state = 'middle'
        self.counter = 0
        self.previousPerception = 0
        self.distChangeCount = 0 
        self.previousVelocity = 0
        self.controlSub = rospy.Subscriber("/gem/ackermann_cmd", AckermannDrive, self.controlCallback)
        
        def normalize(v):
            norm = np.linalg.norm(v)
            if norm == 0: 
                return v
            return v*2 / norm

        def transform(x,y,side="clockwise"):
            if side == "clockwise":
                perpendicular = np.array([y,-x])
                new = normalize(perpendicular) + [x,y]
                return new
            elif side == "anticlockwise":
                perpendicular = np.array([-y,x])
                new = normalize(perpendicular) + [x,y]
                return new

        new_waypoints_right = []
        new_waypoints_left = []
        waypoints = self.waypoint_list
        for i,waypoint in enumerate(waypoints):
            if(i<len(waypoints)-1):
                vector = waypoints[i+1]-waypoint
                new_point = transform(vector[0],vector[1],"clockwise") + waypoint
        #         print(new_point)
                new_waypoints_right.append(new_point)
                new_point = transform(vector[0],vector[1],"anticlockwise") + waypoint
        #         print(new_point)
                new_waypoints_left.append(new_point)
                
        new_waypoints_right = [[new_waypoints_right[0][0],self.waypoint_list[0][1]]] + new_waypoints_right
        new_waypoints_left = [[new_waypoints_left[0][0],self.waypoint_list[0][1]]] + new_waypoints_left
        
        self.new_waypoints_right = np.array(new_waypoints_right)
        self.new_waypoints_left = np.array(new_waypoints_left)

        self.prev_direction = 0
        self.change_counter = 0
        self.waitTimeCount = 0
        self.velocityCounter = 0

    def controlCallback(self,data):
        # print("Speed:",data.speed)
        self.previousVelocity = data.speed
        
    def get_ref_state(self, currState, perceptionInput,pedPosition,distance):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs: 
                currState: ModelState, the current state of vehicle
                perceptionInput: float, currently the distance between vehicle and obstacle in front
            Outputs: reference state position and velocity of the vehicle 
        """
        # positive value means car is moving closer to pedestrian
        differencePosition = perceptionInput - self.previousPerception
        relativeVelocity = 0

        if not math.isnan(differencePosition) and differencePosition!=0:
            # print("Dist Change",self.distChangeCount)
            # print("Previous Distance:",self.previousPerception)    
            # print("Current Distance:",perceptionInput)
            # print("Difference:",differencePosition)
            relativeVelocity = differencePosition/0.1
            # print("Relative Velocity",relativeVelocity)
            # print("Actual Velocity",self.previousVelocity)
            # print("Time Taken for Change:",time,ref_v)
            self.distChangeCount = 0
        
        # print("Current Distance:",perceptionInput)
        # print("Current Distance:",distance)
        # print("Current Distance:",perceptionInput)
        # print("Previous Distance:",self.previousPerception)    
        
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
            # print("Distance To Pedestrian Less than 15")
            # if self.velocityCounter < 1000:
            #     target_x = curr_x
            #     target_y = curr_y
            #     ref_v = 0
            # else:
            #     # curr_x = currState.pose.position.x
            #     # curr_y = currState.pose.position.y
                
            #     # distToTargetX = abs(target_x - curr_x)
            #     # distToTargetY = abs(target_y - curr_y)

            #     # if ((distToTargetX < 0.5 and distToTargetY < 0.5)) or self.counter > 1000:
            #     #     self.counter = 0
            #     #     self.pos_idx += 1
            #     #     self.pos_idx = int(self.pos_idx % len(self.waypoint_list))
            #     #     print("reached",self.waypoint_list[self.pos_idx-1][0],self.waypoint_list[self.pos_idx-1][1],
            #     #         "next",self.waypoint_list[self.pos_idx][0],self.waypoint_list[self.pos_idx][1])
            #     # else:
            #     #     self.counter += 1

            #     if self.counter > 90:
            #         self.counter = 0
            #         self.pos_idx += 1
            #     else:
            #         self.counter += 1


            #     ref_v = 10
            
            # # print("Distance less than 5")
            # # print("front_dist:",front_dist)
            
            # if self.waitTimeCount > 1000:
            #     self.change_counter = 0
            #     self.pos_idx +=1
            #     if self.prev_direction == 0:
            #         print("Distance to pedestrian been < 15 for too Long")
            #         print("Switch lane to right")
            #         self.prev_direction = 1
            #         target_x = self.new_waypoints_right[self.pos_idx][0]
            #         target_y = self.new_waypoints_right[self.pos_idx][1]
            #         print("Target:",target_x,target_y)
            #         print("Current:",curr_x,curr_y)
            #     else:
            #         print("Switch lane to left")
            #         self.prev_direction = 0
            #         target_x = self.new_waypoints_left[self.pos_idx][0]
            #         target_y = self.new_waypoints_left[self.pos_idx][1]
                
            #     self.waitTimeCount = 0
            
            # self.waitTimeCount += 1
            # self.change_counter += 1
            # self.velocityCounter += 1

            # if self.prev_direction == 0:
            #     target_x = self.new_waypoints_left[self.pos_idx][0]
            #     target_y = self.new_waypoints_left[self.pos_idx][1]
            # else:
            #     target_x = self.new_waypoints_right[self.pos_idx][0]
            #     target_y = self.new_waypoints_right[self.pos_idx][1]

            target_x = curr_x
            target_y = curr_y
            ref_v = -1

        else:
            self.velocityCounter = 0
            if self.prev_direction == 0:
                target_x = self.new_waypoints_left[self.pos_idx][0]
                target_y = self.new_waypoints_left[self.pos_idx][1]
            else:
                target_x = self.new_waypoints_right[self.pos_idx][0]
                target_y = self.new_waypoints_right[self.pos_idx][1]

            curr_x = currState.pose.position.x
            curr_y = currState.pose.position.y
            
            distToTargetX = abs(target_x - curr_x)
            distToTargetY = abs(target_y - curr_y)

            if ((distToTargetX < 0.5 and distToTargetY < 0.5)) or self.counter > 90:
                self.counter = 0
                self.pos_idx += 1
                self.pos_idx = int(self.pos_idx % len(self.waypoint_list))
                print("reached",self.waypoint_list[self.pos_idx-1][0],self.waypoint_list[self.pos_idx-1][1],
                    "next",self.waypoint_list[self.pos_idx][0],self.waypoint_list[self.pos_idx][1])
            else:
                self.counter += 1

            if front_dist < 20:

                if(relativeVelocity!=0):
                    ref_v = self.previousVelocity + relativeVelocity
                    # print("Previous,Relative:",self.previousVelocity,relativeVelocity)
                    # print("Ref v:",ref_v)
                else:
                    ref_v = self.previousVelocity

                if relativeVelocity > 0:
                    # print("Speeding Up")
                    wsd = 5
                else:
                    wsd = 5
                    # print("Slowing Down")
                    

            else:
                # print("Speeding Up")
                ref_v = self.previousVelocity
    
        self.distChangeCount += 1
        self.previousPerception = perceptionInput

        return [target_x, target_y, ref_v]
