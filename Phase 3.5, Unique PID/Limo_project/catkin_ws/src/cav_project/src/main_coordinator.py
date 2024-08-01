#!/usr/bin/env python3
#import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from cav_class import CAV
from cav_project.msg import limo_info, QP_solution, ControlInfo, limo_state, limo_state_matrix
from function import calc_qp_info

class MainCoordinator:
    def __init__(self):
        rospy.init_node('main_coordinator', anonymous=True)
        # Uncomment if you need to publish the array data somewhere
        # self.limo_array_pub = rospy.Publisher('/limo_info_array', limo_info_array, queue_size=10)
        # self.limo_array_pub.publish(self.limo_array_data)
        self.limo_state_matrix_pub = rospy.Publisher('/limo_state_matrix', limo_state_matrix, queue_size=10)
        self.rate = rospy.Rate(15)


    def run(self):
        #cav1 = CAV("limo770", False, 'm', 'j')
        #cav2 = CAV("limo155", True, 'g', 's')
        #cav3 = CAV("limo795", True, 't', 'h')
        #cav4 = CAV("limo789", True, 'm', 'j')
        #order_list = [cav1, cav2, cav3, cav4]


        #cav1 = CAV("limo770", False, 't', 'h')
        #cav2 = CAV("limo155", False, 'm', 'j')
        #cav3 = CAV("limo795", True, 'g', 's')
        #cav4 = CAV("limo789", True, 'm', 'j')
        #cav5 = CAV("limo799", True, 'n', 's')
        #order_list = [cav1, cav2, cav3, cav4]


        #cav1 = CAV("limo770", True)
        #cav2 = CAV("limo155", True) # True is main road and merging section
        #cav3 = CAV("limo795", True)
        #cav4 = CAV("limo789", True, 'm', 'j')
        #cav5 = CAV("limo799", False)
        #order_list = [cav1, cav2]

        cav1 = CAV("limo770", False, 't', 'h')
        cav2 = CAV("limo155", True, 'm', 'j')
        cav3 = CAV("limo795", True, 'g', 's')
        cav4 = CAV("limo789", True, 'm', 'j')
        cav5 = CAV("limo799", True, 'n', 's')
        #order_list = [cav1, cav5, cav2, cav4, cav3, cav6, cav9, cav8,cav7]
        #order_list = [cav5]
        #order_list = [cav5, cav3, cav1]

        #cav1 = CAV("limo770", False, 'm', 'h')
        #cav2 = CAV("limo155", False, 'm', 'j')


        cav6 = CAV("limo780", True)
        cav7 = CAV("limo813", False)
        cav8= CAV("limo777", True)
        cav9 = CAV("limo793", False)
        #order_list = [ cav6]
        order_list = [cav1, cav5, cav2, cav4, cav3, cav6, cav9, cav7,cav8]
        #order_list= [cav1, cav5, cav2, cav4, cav3]
        #cav7 = CAV("limo813", False)
        #order_list = [cav6, cav9, cav8,cav7]
        #order_list= [cav1]
        while not rospy.is_shutdown():
            limo_state_mat = limo_state_matrix()
            for i in range(len(order_list)):
                limo_state_msg = calc_qp_info(order_list, i)
                limo_state_mat.limos.append(limo_state_msg)
            self.limo_state_matrix_pub.publish(limo_state_mat)

            if True: #self.qp_solution_cav1 and self.qp_solution_cav2 and self.cav_info_cav1 and self.cav_info_cav2:
                cav1.run()
                cav2.run()
                cav3.run()
                cav4.run()
                cav5.run()
                cav6.run()
                cav9.run()
                cav8.run()
                cav7.run()

                self.rate.sleep()


if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()
