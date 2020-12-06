import rospy
import numpy as np
import argparse

# from gazebo_msgs.msg import  ModelState
# from controller.controller import VehicleController
from perception.perception import VehiclePerception
from decision.decision import VehicleDecision
import time
from util.waypoint_list import wayPoints
from util.util import euler_to_quaternion, quaternion_to_euler
import pickle
from positionDetector.positionDetector import PositionDetector
from safetyDetector.safetyDetector import SafetyDetector
from locationGPS.locationGPS import locationGPS
import sys

def run_model(model_name,runVehicle):
    resolution = 0.1
    rospy.init_node("gem1_dynamics")
    rate = rospy.Rate(10)  # 100 Hz    

    perceptionModule = VehiclePerception(model_name)
    decisionModule = VehicleDecision('./waypoints')
    # controlModule = VehicleController(model_name)
    # posDetector = PositionDetector(resolution=resolution)
    # safety = SafetyDetector(10, resolution)
    gpsLoc = locationGPS()

    allGPS = []
    i = 0
    print("Outside While loop")
    while not rospy.is_shutdown():
        # res = sensors.lidarReading()
        print("distance")
        # print(res)
          # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        # currState =  perceptionModule.gpsReading()
        perceptionResult = perceptionModule.lidarReading()

        # pedImgPosition = posDetector.getPosition()

        
        # safe, pedPosition, distance = safety.checkSafety(currState, pedImgPosition)
        print("distance",perceptionResult)
        # refState = decisionModule.get_ref_state(currState, perceptionResult, pedPosition, distance)
        # if runVehicle == "run":
        # controlModule.execute(currState, refState)

        if(model_name=="highbay"):
            allGPS.append(gpsLoc.returnGPSCoord())
            if(i==8000):
                pickle.dump(allGPS, open("gpsDump", "wb"))
                print("Dumped")
            i += 1 

        # rate.spin()

if __name__ == "__main__":

    runVehicle = "run"
    model_name = "gem"
    if(len(sys.argv)>1):
        if(sys.argv[1] == "gem" or sys.argv[1] == "highbay"):
            model_name = sys.argv[1]
    if(len(sys.argv)>2):
        if(sys.argv[2] == "run" or sys.argv[2] == "stationary"):
            runVehicle = sys.argv[2]
    run_model(model_name,runVehicle)
    print("In the Main fucntion")

    
