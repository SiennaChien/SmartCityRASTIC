#!/usr/bin/env python3
#import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from cav_class import CAV
from cav_project.msg import limo_info, QP_solution, ControlInfo, limo_state, limo_state_matrix
from function import search_ahead, search_previous, calc_distance, calc_qp_info

class MainCoordinator:
    def __init__(self):
        rospy.init_node('main_coordinator', anonymous=True)
        # Uncomment if you need to publish the array data somewhere
        # self.limo_array_pub = rospy.Publisher('/limo_info_array', limo_info_array, queue_size=10)
        # self.limo_array_pub.publish(self.limo_array_data)
        self.limo_state_matrix_pub = rospy.Publisher('/limo_state_matrix', limo_state_matrix, queue_size=10)
        self.rate = rospy.Rate(15)

    def run(self):
        cav1 = CAV("limo770", False)
        cav2 = CAV("limo155", True)
        cav3 = CAV("limo795", True)

        order_list = [cav2, cav3, cav1]

        while not rospy.is_shutdown():
            limo_state_mat = limo_state_matrix()
            for i in range(len(order_list)):
                limo_state_msg = calc_qp_info(order_list, i)
                limo_state_mat.limos.append(limo_state_msg)
            self.limo_state_matrix_pub.publish(limo_state_mat)

            if True: #cav1.qp_solution_limo770 and cav2.qp_solution_limo155 and cav3.qp_solution_limo795 and \
                #cav1.cav_info_limo770 and cav2.cav_info_limo155 and cav3.cav_info_limo795:
                cav1.run()
                cav2.run()
                cav3.run()
                self.rate.sleep()


if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()
