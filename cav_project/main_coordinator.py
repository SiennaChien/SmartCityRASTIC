#!/usr/bin/env python3
import rospy
from cav_project.msg import limo_info, QP_solution
from std_msgs.msg import String

class MainCoordinator:
    def __init__(self):
        rospy.init_node('main_coordinator', anonymous=True)
        self.qp_solution_cav1_sub = rospy.Subscriber('/qp_solution_cav1', QP_solution, self.qp_solution_cav1_callback)
        self.qp_solution_cav2_sub = rospy.Subscriber('/qp_solution_cav2', QP_solution, self.qp_solution_cav2_callback)
        self.cav_info_cav1_sub = rospy.Subscriber('/limo_info_cav1', limo_info, self.cav_info_cav1_callback)
        self.cav_info_cav2_sub = rospy.Subscriber('/limo_info_cav2', limo_info, self.cav_info_cav2_callback)
        self.merged_cav_info_pub = rospy.Publisher('/merged_cav_info', String, queue_size=10)
        
        self.qp_solution_cav1 = None
        self.qp_solution_cav2 = None
        self.cav_info_cav1 = None
        self.cav_info_cav2 = None
        
        self.rate = rospy.Rate(10)

    def qp_solution_cav1_callback(self, msg):
        self.qp_solution_cav1 = msg

    def qp_solution_cav2_callback(self, msg):
        self.qp_solution_cav2 = msg

    def cav_info_cav1_callback(self, msg):
        self.cav_info_cav1 = msg

    def cav_info_cav2_callback(self, msg):
        self.cav_info_cav2 = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.qp_solution_cav1 and self.qp_solution_cav2 and self.cav_info_cav1 and self.cav_info_cav2:
                merged_info = String()
                merged_info.data = f"CAV1 - QP Solution: {self.qp_solution_cav1.u.data}, Position: ({self.cav_info_cav1.x.data}, {self.cav_info_cav1.y.data}) | " \
                                   f"CAV2 - QP Solution: {self.qp_solution_cav2.u.data}, Position: ({self.cav_info_cav2.x.data}, {self.cav_info_cav2.y.data})"
                self.merged_cav_info_pub.publish(merged_info)
            self.rate.sleep()

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()
