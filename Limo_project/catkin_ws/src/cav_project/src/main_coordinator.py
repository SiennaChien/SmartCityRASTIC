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
        
        self.qp_solution_cav1 = QP_solution()
        self.qp_solution_cav2 = QP_solution()
        self.cav_info_cav1 = limo_info()
        self.cav_info_cav2 = limo_info()
        
        self.qp_solution_cav1_sub = rospy.Subscriber('/qp_solution_cav1', QP_solution, self.qp_solution_cav1_callback)
        self.qp_solution_cav2_sub = rospy.Subscriber('/qp_solution_cav2', QP_solution, self.qp_solution_cav2_callback)
        self.cav_info_cav1_sub = rospy.Subscriber('/limo_info_cav1', limo_info, self.cav_info_cav1_callback)
        self.cav_info_cav2_sub = rospy.Subscriber('/limo_info_cav2', limo_info, self.cav_info_cav2_callback)
        self.control_info_pub_cav1 = rospy.Publisher('/control_info_cav1', ControlInfo, queue_size=10)
        self.control_info_pub_cav2 = rospy.Publisher('/control_info_cav2', ControlInfo, queue_size=10)

        self.rate = rospy.Rate(10)

        # PID state variables
        self.e_prev_lateral_cav1 = 0
        self.e_int_lateral_cav1 = 0
        self.e_prev_lateral_cav2 = 0
        self.e_int_lateral_cav2 = 0
        self.e_prev_longitudinal_cav1 = 0
        self.e_int_longitudinal_cav1 = 0
        self.e_prev_longitudinal_cav2 = 0
        self.e_int_longitudinal_cav2 = 0
        self.delta_t = 0.05

    def qp_solution_cav1_callback(self, msg):
        self.qp_solution_cav1 = msg

    def qp_solution_cav2_callback(self, msg):
        self.qp_solution_cav2 = msg

    def cav_info_cav1_callback(self, msg):
        self.cav_info_cav1 = msg

    def cav_info_cav2_callback(self, msg):
        self.cav_info_cav2 = msg

    def pid_lateral_controller(self, lateral_error, e_prev, e_int):
        kp = -0.0015
        ki = -0.000045
        kd = -0.0017
        e_int += lateral_error * self.delta_t
        e_der = (lateral_error - e_prev) / self.delta_t
        steering_angle = kp * lateral_error + ki * e_int + kd * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def pid_longitudinal_controller(self, desired_velocity, actual_velocity, e_prev, e_int):
        kp = 0.1
        ki = 0.01
        kd = 0.05
        error = desired_velocity - actual_velocity
        e_int += error * self.delta_t
        e_der = (error - e_prev) / self.delta_t
        control_input = kp * error + ki * e_int + kd * e_der
        control_input = max(min(control_input, 1), -1)
        return control_input, error, e_int

    def run(self):
        while not rospy.is_shutdown():
            if self.qp_solution_cav1 and self.qp_solution_cav2 and self.cav_info_cav1 and self.cav_info_cav2:
                # Calculate desired velocities using QP solutions
                desired_velocity_cav1 = self.cav_info_cav1.vel.data + self.qp_solution_cav1.u * 0.05
                desired_velocity_cav2 = self.cav_info_cav2.vel.data + self.qp_solution_cav2.u * 0.05

                # PID Lateral Control for CAV1
                if self.cav_info_cav1.d1.data**2 + self.cav_info_cav1.d2.data**2 != 0:
                    lateral_error_cav1 = -(self.cav_info_cav1.d1.data * self.cav_info_cav1.d2.data + self.cav_info_cav1.d2.data * self.cav_info_cav1.d2.data + self.cav_info_cav1.d2.data) / ((self.cav_info_cav1.d1.data**2 + self.cav_info_cav1.d2.data**2)**0.5)
                else:
                    lateral_error_cav1 = 0
                steering_angle_cav1, self.e_prev_lateral_cav1, self.e_int_lateral_cav1 = self.pid_lateral_controller(lateral_error_cav1, self.e_prev_lateral_cav1, self.e_int_lateral_cav1)

                # PID Longitudinal Control for CAV1
                actual_velocity_cav1 = self.cav_info_cav1.vel.data
                control_input_cav1, self.e_prev_longitudinal_cav1, self.e_int_longitudinal_cav1 = self.pid_longitudinal_controller(desired_velocity_cav1, actual_velocity_cav1, self.e_prev_longitudinal_cav1, self.e_int_longitudinal_cav1)

                # Publish control info for CAV1
                control_info_cav1 = ControlInfo()
                control_info_cav1.steering_angle = steering_angle_cav1
                control_info_cav1.desired_velocity = desired_velocity_cav1
                control_info_cav1.control_input = control_input_cav1
                self.control_info_pub_cav1.publish(control_info_cav1)

                # PID Lateral Control for CAV2
                if self.cav_info_cav2.d1.data**2 + self.cav_info_cav2.d2.data**2 != 0:
                    lateral_error_cav2 = -(self.cav_info_cav2.d1.data * self.cav_info_cav2.d2.data + self.cav_info_cav2.d2.data * self.cav_info_cav2.d2.data + self.cav_info_cav2.d2.data) / ((self.cav_info_cav2.d1.data**2 + self.cav_info_cav2.d2.data**2)**0.5)
                else:
                    lateral_error_cav2 = 0
                steering_angle_cav2, self.e_prev_lateral_cav2, self.e_int_lateral_cav2 = self.pid_lateral_controller(lateral_error_cav2, self.e_prev_lateral_cav2, self.e_int_lateral_cav2)

                # PID Longitudinal Control for CAV2
                actual_velocity_cav2 = self.cav_info_cav2.vel.data
                control_input_cav2, self.e_prev_longitudinal_cav2, self.e_int_longitudinal_cav2 = self.pid_longitudinal_controller(desired_velocity_cav2, actual_velocity_cav2, self.e_prev_longitudinal_cav2, self.e_int_longitudinal_cav2)

                # Publish control info for CAV2
                control_info_cav2 = ControlInfo()
                control_info_cav2.steering_angle = steering_angle_cav2
                control_info_cav2.desired_velocity = desired_velocity_cav2
                control_info_cav2.control_input = control_input_cav2
                self.control_info_pub_cav2.publish(control_info_cav2)

            self.rate.sleep()

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()
