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
        #cav3 = CAV("limo795", True)

        order_list = [cav2, cav1]
        search_info = []

        for i in range(len(order_list)):
            cav = order_list[i]
            current = cav.current
            search_info_dic = {
                "cav_object": cav,
                "cav_name": cav.ID,
                "current_line": cav.lines[current],
                "current_start_pt": cav.points[current],
                "current_end_pt": cav.points[(current+1) % len(cav.points)],
                "current_pos": [cav.position_x, cav.position_z],
            }
            search_info.append(search_info_dic)
        while not rospy.is_shutdown():
            search_info = []
            for i in range(len(order_list)):
                cav = order_list[i]
                current = cav.current
                search_info_dic = {
                    "cav_object": cav,
                    "cav_name": cav.ID,
                    "current_line": cav.lines[current],
                    "current_start_pt": cav.points[current],
                    "current_end_pt": cav.points[(current+1) % len(cav.points)],
                    "current_pos": [cav.position_x, cav.position_z],
                }
                print("cav", cav.ID, "current index", current, "next index", cav.next)
                search_info.append(search_info_dic)
            limo_state_mat = limo_state_matrix()
            for i in range(len(order_list)):
                limo_state_msg = calc_qp_info(search_info, i)
                limo_state_mat.limos.append(limo_state_msg)
            self.limo_state_matrix_pub.publish(limo_state_mat)
            if True: #self.qp_solution_cav1 and self.qp_solution_cav2 and self.cav_info_cav1 and self.cav_info_cav2:
                cav1.run()
                cav2.run()
                self.rate.sleep()



# if len(limo_state_matrix_msg.limos) > 0:
#     # Access the d2 value of the first limo in the limos array
#     d2_value = limo_state_matrix_msg.limos[0].d2
#     print("d2 value of the first limo:", d2_value)
# else:
#     print("No limos available in limo_state_matrix_msg")

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()
