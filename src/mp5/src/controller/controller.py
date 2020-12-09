import rospy
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from util.util import euler_to_quaternion, quaternion_to_euler

class VehicleController():

    def __init__(self, model_name='gem'):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/" + model_name + "/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.model_name = model_name

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and 
            the target state to compute low-level control input to the vehicle
            Inputs: 
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)
        
        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]
        
        k_s = 0.1
        k_ds = 1
        k_n = 0.1

        #compute errors
        xError = (target_x - currentPose.pose.position.x) * np.cos(currentEuler[2]) + (target_y - currentPose.pose.position.y) * np.sin(currentEuler[2])
        yError = -(target_x - currentPose.pose.position.x) * np.sin(currentEuler[2]) + (target_y - currentPose.pose.position.y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose.twist.linear.x**2 + currentPose.twist.linear.y**2)
        vError = target_v - curr_v
        # print("TaRgetV,CurrV",target_v,curr_v)
        # print("Target X:",xError)
        delta = k_n*yError
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError*k_s + vError*k_ds
            # v = target_v
            # print("Velcithy with target > 0",v)
        else:
            v = xError*k_s - 0.05*k_ds
            # if v <= 0:
            #     v = 0
                 

        #Send computed control input to vehicle
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        # print("V:",v)
        newAckermannCmd.steering_angle = delta
        self.controlPub.publish(newAckermannCmd)

# import rospy
# import numpy as np
# import argparse
# import math

# from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
# from std_msgs.msg import String, Bool, Float32, Float64, Char
# from pynput.keyboard import Key, Listener, KeyCode

# gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
# enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
# accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
# brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

# enabled = False
# accel_flag = False
# gear_cmd = PacmodCmd()
# accel_cmd = PacmodCmd()
# brake_cmd = PacmodCmd()


    
# def speed_control(data):    
#     accel_cmd = PacmodCmd()
    
#     global accel_flag
#     print(accel_flag, "accel")
#     if accel_flag == False:

#         accel_cmd.f64_cmd = 0
#         accel_pub.publish(accel_cmd)

#     else:
#         brake_msg = PacmodCmd()
#         brake_msg.f64_cmd = 0.0
#         brake_pub.publish(brake_msg)

#         accel_cmd.f64_cmd = 0.4
#         accel_pub.publish(accel_cmd)
#         pass



# def on_press(key):
#     global enabled 
#     global gear_cmd
#     global accel_cmd
#     global brake_cmd
#     global accel_flag

#     if key == KeyCode.from_char('f'):
#         print('ACCELERATING')
#         accel_flag = True
#         brake_cmd.f64_cmd = 0.0
#     if key == KeyCode.from_char('s'):
#         print('STOPPING')
#         accel_flag = False
#         brake_cmd.f64_cmd = 0.5
#     if key == Key.esc:
#         return False
#     if key == KeyCode.from_char('q'):
#         print('DISENGAGED')
#         enabled = False
#         accel_cmd.enable = False
#         accel_cmd.clear = True
#         accel_cmd.ignore = True
#         brake_cmd.enable = False
#         brake_cmd.clear = True
#         brake_cmd.ignore = True
#         gear_cmd.ui16_cmd = 2
#     if key == KeyCode.from_char('p'):
#         print('ENGAGED')
#         enabled = True
#         accel_cmd.enable = True
#         accel_cmd.clear = False
#         accel_cmd.ignore = False
#         brake_cmd.enable = True
#         brake_cmd.clear = False
#         brake_cmd.ignore = False
#         accel_cmd.f64_cmd = 0.0
#         brake_cmd.f64_cmd = 0.5
#         gear_cmd.ui16_cmd = 2
#     if key == KeyCode.from_char('n'):
#         print('GEAR: NEUTRAL')
#         gear_cmd.ui16_cmd = 2
#     if key == KeyCode.from_char('d'):
#         print('GEAR: DRIVE')
#         gear_cmd.ui16_cmd = 3
#     if key == KeyCode.from_char('r'):
#         print('GEAR: REVERSE')
#         gear_cmd.ui16_cmd = 1

#     enable_pub.publish(Bool(enabled))
#     gear_pub.publish(gear_cmd)
#     brake_pub.publish(brake_cmd)


# # .32 threshold

# previousDistance = 0
# def distanceCallback(distance):
    
#     global previousDistance
#     global enabled 
#     global gear_cmd
#     global accel_cmd
#     global brake_cmd
#     global accel_flag
    
#     if not enabled or not accel_flag:
#         return 
    
#     # print("in Distance callback ",distance)
#     # print("Previous:",previousDistance)
#     if(previousDistance-float(distance.data)!=0):
#         print("Rel Velocity:",(previousDistance-distance.data)/0.1)

#     previousDistance = float(distance.data)
    
#     if distance.data < 15:
#         # print("Settung cmd to 0")
#         accel_cmd.f64_cmd = 0.0
#         accel_pub.publish(accel_cmd)
#     else:
#         accel_cmd.f64_cmd = 0.4
#         accel_pub.publish(accel_cmd)



# def run_model(model_name): 

#     #controller part
#     rospy.init_node('GEM_Control', anonymous=True)
#     # rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_control)
#     enable_pub.publish(Bool(False))

#     listener = Listener(on_press=on_press)
#     listener.start()

#     rospy.Subscriber("/mp5/Distance",Float64,distanceCallback)

#     rate = rospy.Rate(10)

#     while not rospy.is_shutdown():

#         # TODO 
#         rate.sleep()  # Wait a while before trying to get a new state        

# if __name__ == "__main__":
#     run_model('gem')

#     rospy.spin()

    
