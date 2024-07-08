#!/usr/bin/env python3

from pylimo import limo
import rospy
from ackermann_msgs.msg import AckermannDrive
import time
import numpy as np
import pickle

class listener:
    def __init__(self):
        self.data = None
        self.xprev = 0
        self.yprev = 0

    def callback(self, data):
        self.data = data

    def listener(self):
        rospy.init_node("vs_listener", anonymous=True)
        rospy.Subscriber("vel_steer_limo770", AckermannDrive, self.callback)


if __name__ == '__main__':
    listener_ins = listener()
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
            listener_ins.data.speed
            limo.SetMotionCommand(linear_vel=listener_ins.data.speed, steering_angle=listener_ins.data.steering_angle)
            print("Limo velocity command:", listener_ins.data.speed, "Limo steering:", listener_ins.data.steering_angle )
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
