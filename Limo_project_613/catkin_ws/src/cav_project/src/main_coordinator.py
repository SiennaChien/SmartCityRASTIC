#!/usr/bin/env python3
import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from cav_class import CAV
from cav_project.msg import limo_info, QP_solution, ControlInfo

class MainCoordinator:
    def __init__(self):
        rospy.init_node('main_coordinator', anonymous=True)
        # Uncomment if you need to publish the array data somewhere
        #self.limo_array_pub = rospy.Publisher('/limo_info_array', limo_info_array, queue_size=10)
        # self.limo_array_pub.publish(self.limo_array_data)

        #self.control_info_pub_cav1 = rospy.Publisher('/control_info_cav1', ControlInfo, queue_size=10)
        #self.control_info_pub_cav2 = rospy.Publisher('/control_info_cav2', ControlInfo, queue_size=10)

        self.rate = rospy.Rate(10)

    def ackermann_callback_cav1(self, msg):
        self.steering_angle_cav1 = msg.steering_angle
        self.desired_velocity_cav1 = msg.speed

    def ackermann_callback_cav2(self, msg):
        self.steering_angle_cav2 = msg.steering_angle
        self.desired_velocity_cav2 = msg.speed

    def run(self):

        cav1 = CAV("limo770", True)
        cav2 = CAV("limo155", True)

        while not rospy.is_shutdown():
            #if True: #self.qp_solution_cav1 and self.qp_solution_cav2 and self.cav_info_cav1 and self.cav_info_cav2:
                cav1.run()
                cav2.run()
                self.rate.sleep()

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()