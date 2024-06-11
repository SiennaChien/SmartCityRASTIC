#!/usr/bin/env python3
import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from cvxopt.solvers import qp
from cvxopt import matrix, solvers
from scipy.integrate import odeint
from cav_class import CAV
from cav_project.msg import limo_info, QP_solution,ControlInfo
from std_msgs.msg import String



class MainCoordinator:
    def __init__(self):
        rospy.init_node('main_coordinator', anonymous=True)

        # Initialize CAV objects
        self.CAV1 = CAV("limo770")
        self.CAV2 = CAV("limo155")

        # Setup Subscribers for QP solutions and Limo Info
        self.qp_solution_cav1_sub = rospy.Subscriber('/qp_solution_cav1', QP_solution, self.qp_solution_cav1_callback)
        self.qp_solution_cav2_sub = rospy.Subscriber('/qp_solution_cav2', QP_solution, self.qp_solution_cav2_callback)
        self.cav_info_cav1_sub = rospy.Subscriber('/limo_info_cav1', limo_info, self.cav_info_cav1_callback)
        self.cav_info_cav2_sub = rospy.Subscriber('/limo_info_cav2', limo_info, self.cav_info_cav2_callback)

        # Setup Publishers for control commands
        self.control_pub_cav1 = rospy.Publisher('control_info_cav1', ControlInfo, queue_size=10)
        self.control_pub_cav2 = rospy.Publisher('control_info_cav2', ControlInfo, queue_size=10)

        # Data holders for PID control values and previous errors
        self.setup_control_variables()

        self.rate = rospy.Rate(10)

    def setup_control_variables(self):
        # Initialize control variables for PID
        self.control_vars = {
            'cav1': {'prev_lateral': 0, 'int_lateral': 0, 'prev_long': 0, 'int_long': 0},
            'cav2': {'prev_lateral': 0, 'int_lateral': 0, 'prev_long': 0, 'int_long': 0}
        }

    def qp_solution_cav1_callback(self, msg):
        self.qp_solution_cav1 = msg

    def qp_solution_cav2_callback(self, msg):
        self.qp_solution_cav2 = msg

    def cav_info_cav1_callback(self, msg):
        self.cav_info_cav1 = msg

    def cav_info_cav2_callback(self, msg):
        self.cav_info_cav2 = msg

    def pid_controller(self, kp, ki, kd, error, prev_error, integral, delta_t):
        integral += error * delta_t
        derivative = (error - prev_error) / delta_t
        control = kp * error + ki * integral + kd * derivative
        return control, integral

    def run(self):
        delta_t = 0.05 # Control update period

        while not rospy.is_shutdown():
            if self.cav_info_cav1 and self.qp_solution_cav1:
                lateral_error_cav1 = self.calculate_lateral_error(self.cav_info_cav1)
                desired_velocity_cav1 = self.calculate_desired_velocity(self.qp_solution_cav1, self.cav_info_cav1)

                steering_cav1, self.control_vars['cav1']['int_lateral'] = self.pid_controller(
                    0.0015, 0.000045, 0.0017, lateral_error_cav1,
                    self.control_vars['cav1']['prev_lateral'],
                    self.control_vars['cav1']['int_lateral'], delta_t
                )
                velocity_control_cav1, self.control_vars['cav1']['int_long'] = self.pid_controller(
                    0.1, 0.01, 0.05, desired_velocity_cav1 - self.cav_info_cav1.vel.data,
                    self.control_vars['cav1']['prev_long'],
                    self.control_vars['cav1']['int_long'], delta_t
                )

                # Publish control info
                control_msg_cav1 = ControlInfo()
                control_msg_cav1.steering_angle = steering_cav1
                control_msg_cav1.velocity = velocity_control_cav1
                self.control_pub_cav1.publish(control_msg_cav1)

                # Update previous errors
                self.control_vars['cav1']['prev_lateral'] = lateral_error_cav1
                self.control_vars['cav1']['prev_long'] = desired_velocity_cav1 - self.cav_info_cav1.vel.data

            if self.cav_info_cav2 and self.qp_solution_cav2:
                lateral_error_cav2 = self.calculate_lateral_error(self.cav_info_cav2)
                desired_velocity_cav2 = self.calculate_desired_velocity(self.qp_solution_cav2, self.cav_info_cav2)

                steering_cav2, self.control_vars['cav2']['int_lateral'] = self.pid_controller(
                    0.0015, 0.000045, 0.0017, lateral_error_cav2,
                    self.control_vars['cav2']['prev_lateral'],
                    self.control_vars['cav2']['int_lateral'], delta_t
                )
                velocity_control_cav2, self.control_vars['cav2']['int_long'] = self.pid_controller(
                    0.1, 0.01, 0.05, desired_velocity_cav2 - self.cav_info_cav2.vel.data,
                    self.control_vars['cav2']['prev_long'],
                    self.control_vars['cav2']['int_long'], delta_t
                )

                # Publish control info
                control_msg_cav2 = ControlInfo()
                control_msg_cav2.steering_angle = steering_cav2
                control_msg_cav2.velocity = velocity_control_cav2
                self.control_pub_cav2.publish(control_msg_cav2)

                # Update previous errors
                self.control_vars['cav2']['prev_lateral'] = lateral_error_cav2
                self.control_vars['cav2']['prev_long'] = desired_velocity_cav2 - self.cav_info_cav2.vel.data

            self.rate.sleep()

    def calculate_lateral_error(self, cav_info):
        return -(cav_info.d1.data * cav_info.d2.data + cav_info.d2.data * cav_info.d2.data + cav_info.d2.data) / ((cav_info.d1.data**2 + cav_info.d2.data**2)**0.5)

    def calculate_desired_velocity(self, qp_solution, cav_info):
        return cav_info.vel.data + qp_solution.u.data * 0.05

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()

