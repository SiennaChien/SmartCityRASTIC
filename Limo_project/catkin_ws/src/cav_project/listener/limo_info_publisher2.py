#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pylimo import limo
from ackermann_msgs.msg import AckermannDrive
import time
import numpy as np
import pickle
from cav_project.msg import limo_info, limo_info_array, ControlInfo, QP_solution


class LimoInfoPublisher:
    def __init__(self):
        rospy.init_node('limo_info_publisher2')
        self.info_pub_ = rospy.Publisher('/limo_info_cav2', limo_info, queue_size=10)
        self.odom_sub_cav2 = rospy.Subscriber('/odom_cav2', Odometry, self.odom_callback_cav2)
        self.control_info_sub_cav2 = rospy.Subscriber("control_info_cav2", ControlInfo, self.control_info_callback)

        self.limo_data_cav2 = limo_info()
        self.control_info_cav2 = ControlInfo()

        self.cont
        self.rate = rospy.Rate(10)

    def control_info_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.control_info_cav2 = data

    def publish_info(self):
        self.info_pub_cav2.publish(self.limo_data_cav2)

    def odom_callback_cav2(self, msg):
        self.limo_data_cav2.vel.data = msg.twist.twist.linear.x
        self.publish_info()


    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()




if __name__ == '__main__':
    publisher = LimoInfoPublisher()
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
            limo.SetMotionCommand(linear_vel=publisher.control_info_cav2.desired_velocity, steering_angle=publisher.control_info_cav2.steering_angle)
            print("Limo velocity command:", publisher.control_info_cav2.desired_velocity, "Limo steering:", publisher.control_info_cav2.steering_angle)
            #steering_angle[iter] = limo.GetSteeringAngle()
           # imu_yaw[iter] = limo.GetIMUYawData()
            iter += 1
            time.sleep(0.1)
        else:
            print("[WARNING] Skipping ros messages...")
            if iter>10:
                break
    #outputFile = 'steering.pickle'
    #data = {'imu_yaw':imu_yaw, 'steer_limo':steering_angle}
    #fw = open(outputFile, 'wb')
    #pickle.dump(data, fw)
    #fw.close()

    publisher.publish_info()
