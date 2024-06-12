#!/usr/bin/env python3

from pylimo import limo
import rospy
from ackermann_msgs.msg import AckermannDrive
import time
import numpy as np
import pickle
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cav_project.msg import limo_info, limo_info_array, ControlInfo, QP_solution

class listener:
    def __init__(self):
        self.data = None
        self.xprev = 0
        self.yprev = 0

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.data = data

    def listener(self):
        rospy.Subscriber("control_info_cav2", ControlInfo, self.callback)

    #def publisher(self):
    #    self.limo_info_pub=rospy.Publisher('/limo_info_cav2', limo_info, queue_size=10)


if __name__ == '__main__':
    rospy.init_node("limo_node", anonymous=True)
    listener_ins = listener()
    #publisher_ins = publisher()
    limo= limo.LIMO()
    limo.EnableCommand()
    limo.SetMotionCommand(linear_vel=0, steering_angle=0)
    time.sleep(1)
    steering_angle = np.zeros(250)
    imu_yaw =  np.zeros(250)
    iter = 0
    while True:
        listener_ins.listener()
        if listener_ins.data is not None:
            #listener_ins.data.speed
            #listener to message and set velocity and steering
            limo.SetMotionCommand(linear_vel=listener_ins.data.desired_velocity, steering_angle=listener_ins.data.steering_angle)
            print("Limo velocity command:", listener_ins.data.desired_velocity, "Limo steering:", listener_ins.data.steering_angle )
            #steering_angle[iter] = limo.GetSteeringAngle()
           # imu_yaw[iter] = limo.GetIMUYawData()
            iter += 1
            time.sleep(0.1)
        else:
            print("[WARNING] Skipping ros messages...")
            if iter>10:
                break
    outputFile = 'steering.pickle'
    data = {'imu_yaw':imu_yaw, 'steer_limo':steering_angle}
    fw = open(outputFile, 'wb')
    pickle.dump(data, fw)
    fw.close()
