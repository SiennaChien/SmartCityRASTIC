#!/usr/bin/env python3
import math
import numpy as np
import time
from cvxopt.solvers import qp
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import odeint
from cvxopt import matrix, solvers
from cav_project.msg import limo_info, QP_solution, ControlInfo


class CAV:
    def __init__(self, ID, isMain):
        self.ID = ID
        self.isMain = isMain
        #rospy.init_node('CAV' + self.ID, anonymous=True)
        self.control_info_pub = rospy.Publisher('/control_info_' + self.ID, ControlInfo, queue_size=10)
        self.mocap_sub = rospy.Subscriber('/vrpn_client_node/' + self.ID + '/pose', PoseStamped, self.mocap_callback)
        self.qp_solution_sub = rospy.Subscriber('/qp_solution_' + self.ID, QP_solution, self.qp_solution_callback)
        self.cav_info_sub = rospy.Subscriber('/limo_info_' + self.ID, limo_info, self.cav_info_callback)
        self.rate = rospy.Rate(10)

        # PID state variables
        self.qp_solution = QP_solution()

        self.e_prev_lateral = 0
        self.e_int_lateral = 0
        self.e_prev_longitudinal = 0
        self.e_int_longitudinal = 0
        self.delta_t = 0.1
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.v_min = 0.15
        self.v_max = 1
        self.u_min = -10
        self.u_max = 2
        self.Delta_T = 0.1
        self.Lc = 0.0
        self.phiRearEnd = 1.2
        self.phiLateral = 1.0
        self.deltaSafetyDistance = 300
        self.max_steering_angle = 7000

        self.position_x = 0
        self.position_y = 0
        self.position_z = 0

    def mocap_callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_yaw = 0
        self.Receivedata = 1

    def qp_solution_callback(self, msg):
        self.qp_solution = msg
        print("qp solution callback function was called")

    def cav_info_callback(self, msg):
        self.cav_info = msg

    def generate_map(self, isMain):
        self.right_top_x = 3044
        self.right_top_y = 2536
        self.right_center_x = 2040
        self.right_center_y = 2519
        self.right_bottom_x = 683
        self.right_bottom_y = 2536
        self.left_top_x = 2933
        self.left_top_y = -1980
        self.left_center_x = 1922
        self.left_center_y = -1977
        self.merging_pt_x = 1950
        self.merging_pt_y = -652
        self.lane_width = 450

        self.right_top_activation_range = (self.lane_width / 1.3, self.lane_width * 1.1)
        self.right_center_activation_range = (self.lane_width * 1.1, self.lane_width)
        self.right_bottom_activation_range = (self.lane_width * 1.2, self.lane_width)
        self.left_top_activation_range = (self.lane_width * 1.1, self.lane_width / 1.7)
        self.left_center_activation_range = (self.lane_width / 1.5, self.lane_width)
        self.merging_pt_activation_range = (self.lane_width, self.lane_width / 2)

        self.main_path = self.generate_line(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.merging_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)

        self.merge_path_PID = (-0.0005, -0.00005, -0.001)
        self.main_path_PID = (-0.0005, -0.00005, -0.001)

        if isMain:
            self.points = [(self.right_center_x, self.right_center_y), (self.left_center_x, self.left_center_y)]
            self.lines = [self.main_path]
            self.dists = [self.calc_distance(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)]
            self.PIDs = [self.main_path_PID]
        else:
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y)]
            self.lines = [self.merging_path]
            self.dists = [self.calc_distance(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)]
            self.PIDs = [self.merge_path_PID]

    def generate_line(self, x_1, y_1, x_2, y_2):
        A = -(y_2 - y_1)
        B = -(x_1 - x_2)
        C = -(y_1 * (x_2 - x_1) - (y_2 - y_1) * x_1)
        return A, B, C

    def calc_distance(self, x1, y1, x2, y2):
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return distance

    def pid_lateral_controller(self, lateral_error, e_prev, e_int):
        kp = 0.00003
        ki = 0.00004
        kd = 0.0005
        e_int += lateral_error * self.delta_t
        e_der = (lateral_error - e_prev) / self.delta_t
        steering_angle = kp * lateral_error + ki * e_int + kd * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def pid_longitudinal_controller(self, desired_velocity, actual_velocity, e_prev, e_int):
        kp = 0.001
        ki = 0.001
        kd = 0.05
        error = desired_velocity - actual_velocity
        e_int += error * self.delta_t
        e_der = (error - e_prev) / self.delta_t
        control_input = kp * error + ki * e_int + kd * e_der
        control_input = max(min(control_input, 10), -10)
        return control_input, error, e_int

    def run(self):
        self.generate_map(self.isMain)
        # Calculate desired velocities using QP solutions
        actual_velocity = self.velocity
        desired_velocity =actual_velocity + self.qp_solution.u* 0.1  # Use control input from QP solution
        lateral_error = (self.lines[0][0]*self.position_x + self.lines[0][1]*self.position_z + self.lines[0][2]) / ((self.lines[0][0]**2 + self.lines[0][1]**2)**0.5)
        steering_angle, self.e_prev_lateral, self.e_int_lateral = self.pid_lateral_controller(lateral_error, self.e_prev_lateral, self.e_int_lateral)

        control_input, self.e_prev_longitudinal, self.e_int_longitudinal = self.pid_longitudinal_controller(desired_velocity, actual_velocity, self.e_prev_longitudinal, self.e_int_longitudinal)


        # Print control info
        print("lateral_error: ", lateral_error)
        rospy.loginfo(f"CAV{self.ID} Control Info - Steering Angle: {steering_angle}, Desired Velocity: {desired_velocity}, Control Input: {control_input}")

        # Publish control info
        control_info = ControlInfo()
        control_info.steering_angle = steering_angle
        control_info.desired_velocity = desired_velocity
        control_info.control_input = control_input
        self.control_info_pub.publish(control_info)
        #time.sleep(1/10)
