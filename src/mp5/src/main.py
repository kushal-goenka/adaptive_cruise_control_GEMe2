import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller.controller import VehicleController
from perception.perception import VehiclePerception
from decision.decision import VehicleDecision
import time
from util.waypoint_list import wayPoints
from util.util import euler_to_quaternion, quaternion_to_euler
import pickle
from positionDetector.positionDetector import PositionDetector
from safetyDetector.safetyDetector import SafetyDetector

def run_model(model_name):
    resolution = 0.1
    rospy.init_node("gem1_dynamics")
    rate = rospy.Rate(100)  # 100 Hz    

    perceptionModule = VehiclePerception(model_name)
    decisionModule = VehicleDecision('./waypoints')
    controlModule = VehicleController(model_name)
    posDetector = PositionDetector(resolution=resolution)
    safety = SafetyDetector(10, resolution)

    while not rospy.is_shutdown():
        # res = sensors.lidarReading()
        # print(res)
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  perceptionModule.gpsReading()
        perceptionResult = perceptionModule.lidarReading()

        pedImgPosition = posDetector.getPosition()

        safe, pedPosition, distance = safety.checkSafety(currState, pedImgPosition)
        
        refState = decisionModule.get_ref_state(currState, perceptionResult, pedPosition, distance)

        controlModule.execute(currState, refState)

if __name__ == "__main__":
    run_model('gem')
    