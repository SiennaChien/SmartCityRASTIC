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
        self.control_info_pub = rospy.Publisher('/control_info_' + self.ID, ControlInfo, queue_size=1)
        self.mocap_sub = rospy.Subscriber('/vrpn_client_node/' + self.ID + '/pose', PoseStamped, self.mocap_callback)
        self.qp_solution_sub = rospy.Subscriber('/qp_solution_' + self.ID, QP_solution, self.qp_solution_callback)
        self.cav_info_sub = rospy.Subscriber('/limo_info_' + self.ID, limo_info, self.cav_info_callback)
        #self.rate = rospy.Rate(25)
        self.generate_map(self.isMain)

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

        self.phiRearEnd = 1.8
        self.phiLateral = 1.8
        self.deltaSafetyDistance = 0.3
        self.max_steering_angle = 7000

        self.position_x = 0
        self.position_y = 0
        self.position_z = 0

        #for run()
        self.lateral_error = 0
        self.desired_velocity = 0.15
        self.within_critical_range = False
        self.line_changed = True
        self.current = 0
        self.next = 1
        self.current_line = self.lines[self.current]
        self.current_end_pt = self.points[self.next]

    def mocap_callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_yaw = 0
        self.Receivedata = 1
    def qp_solution_callback(self, msg):
        self.qp_solution = msg

    def cav_info_callback(self, msg):
        self.cav_info = msg
        self.velocity = self.cav_info.vel.data

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
        self.merging_pt_x = 1825
        self.merging_pt_y = -652
        self.lane_width = 450

        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.main_path = self.generate_line(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.return_first = self.generate_line(self.left_center_x, self.left_center_y, self.left_top_x, self.left_top_y)
        self.return_second = self.generate_line(self.left_top_x, self.left_top_y, self.right_top_x, self.right_top_y)
        self.return_third = self.generate_line(self.right_top_x, self.right_top_y, self.right_center_x, self.right_center_y)
        self.merging_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)
        self.off_path = self.generate_line(self.right_top_x, self.right_top_y, self.right_bottom_x, self.right_bottom_y) #return third to off path

        #data points that characterize each circle for the corners - center x, center y, radius. Each variable is a tuple (A, B, C)
        self.right_top_circle = (self.right_top_x - self.lane_width, self.right_top_y - self.lane_width, self.lane_width/1.5)
        self.right_center_circle = (self.right_center_x + self.lane_width, self.right_center_y - self.lane_width, self.lane_width/1.5)
        self.right_bottom_circle = (self.right_bottom_x + self.lane_width, self.right_bottom_y - self.lane_width, self.lane_width/2)
        self.left_top_circle = (self.left_top_x - self.lane_width, self.left_top_y + self.lane_width, self.lane_width/1.3)
        self.left_center_circle = (self.left_center_x + self.lane_width, self.left_center_y + self.lane_width, self.lane_width/1.5)
        self.merging_circle = (self.merging_pt_x + self.lane_width, self.merging_pt_y - self.lane_width, self.lane_width) #in practice this is not use

        #the ranges near each corner that activates the circle path for the limo to follow
        self.right_top_activation_range = (self.lane_width / 1.3, self.lane_width * 0.8)
        self.right_center_activation_range = (self.lane_width * 0.9, self.lane_width * 0.6)
        self.right_bottom_activation_range = (self.lane_width * 1.2, self.lane_width * 0.9)
        self.left_top_activation_range = (self.lane_width * 1.1, self.lane_width / 1.8)
        self.left_center_activation_range = (self.lane_width / 1.5, self.lane_width * 1.3)
        self.merging_pt_activation_range = (self.lane_width * 1, self.lane_width * 0.5)

        #self.right_top_activation_range = (self.lane_width * 0.8, self.lane_width * 1)
        #self.right_center_activation_range = (self.lane_width * 1, self.lane_width * 0.8)
        #self.right_bottom_activation_range = (self.lane_width * 1, self.lane_width*1.2)
        #self.left_top_activation_range = (self.lane_width * 0.5, self.lane_width / 1.8)
        #self.left_center_activation_range = (self.lane_width / 1, self.lane_width * 1)
        #self.merging_pt_activation_range = (self.lane_width * 0.7, self.lane_width * 0.5)

        #PID values of each line, each element is a tuple (kp, ki, kd)
        self.merge_path_PID = (0.0005, 0.00005, 0.001) #0.0005, -0.00003, -0.003)
        self.main_path_PID = (0.0005, 0.00005, 0.001)
        self.main_path2_PID = (0.0005, 0.00005, 0.001)
        self.return_first_PID = (0.0005, 0.00005, 0.001)
        self.return_second_PID = (0.0005, 0.00005, 0.001)
        self.return_third_PID = (0.0005, 0.00005, 0.001)# values from tuuning qith qp node 0.00003, 0.00004, 0.0005

        #PID values of each circle, each element is a tuple (kp, ki, kd)
        self.right_bottom_circle_PID = (-0.50, -0.0045, -0.037)
        self.right_center_circle_PID = (-0.56, -0.000045, -0.037)
        self.left_center_circle_PID = (-0.56, -0.00045, -0.037)
        self.left_top_circle_PID = (-0.55, -0.00045, -0.050)
        self.right_top_circle_PID = (-0.55, -0.00045, -0.037)
        self.merging_circle_PID = (-0.0005, -0.00045, -0.003)

        self.merge_path_dist = self.calc_distance(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)
        self.main1_dist = self.calc_distance(self.right_center_x, self.right_center_y, self.merging_pt_x, self.merging_pt_y)
        self.main2_dist = self.calc_distance(self.merging_pt_x, self.merging_pt_y, self.left_center_x, self.left_center_y)
        self.return_first_dist = self.calc_distance(self.left_center_x, self.left_center_y, self.left_top_x, self.left_top_y)
        self.return_second_dist = self.calc_distance(self.left_top_x, self.left_top_y, self.right_top_x, self.right_top_y)
        self.return_third_dist = self.calc_distance(self.right_top_x, self.right_top_y, self.right_center_x, self.right_center_y)


        if isMain: #if tratehe limo runs along the main path
            #array to store all points, in order of traversal
            self.points = [(self.right_center_x, self.right_center_y), (self.merging_pt_x, self.merging_pt_y), (self.left_center_x, self.left_center_y),
                        (self.left_top_x, self.left_top_y), (self.right_top_x, self.right_top_y)]
            #array to store all lines, in order of traversal
            self.lines = [self.main_path, self.main_path, self.return_first, self.return_second, self.return_third]
            #the activation range of the corners, in order of traversal
            self.ranges = [self.right_center_activation_range, self.merging_pt_activation_range, self.left_center_activation_range,
                           self.left_top_activation_range, self.right_top_activation_range]
            #array to store the circles for the corners, in order of traversal
            self.circles = [self.right_center_circle, self.merging_circle, self.left_center_circle, self.left_top_circle, self.right_top_circle]
            #array to store PID values of each line, in order of traversal, each element is a tuple (kp, ki, kd)
            self.PIDs = [self.main_path_PID, self.main_path_PID, self.return_first_PID, self.return_second_PID, self.return_third_PID]
            #array to store PID values of each circle, in order of traversal, each element is a tuple (kp, ki, kd)
            self.curve_PIDs = [self.right_center_circle_PID, self.merging_circle_PID, self.left_center_circle_PID, self.left_top_circle_PID, self.right_top_circle_PID]
            self.dist = [self.main1_dist, self.main2_dist, self.return_first_dist, self.return_second_dist, self.return_third_dist]

        else: #if the limo runs along the merging path
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y), (self.left_center_x, self.left_center_y),
                        (self.left_top_x, self.left_top_y), (self.right_top_x, self.right_top_y)]
            self.lines = [self.merging_path, self.main_path, self.return_first, self.return_second, self.off_path]
            self.PIDs = [self.merge_path_PID, self.main_path2_PID, self.return_first_PID, self.return_second_PID, self.return_third_PID]
            self.ranges = [self.right_bottom_activation_range, self.merging_pt_activation_range,
                           self.left_center_activation_range, self.left_top_activation_range, self.right_top_activation_range]
            self.circles = [self.right_bottom_circle, self.merging_circle, self.left_center_circle, self.left_top_circle, self. right_top_circle]
            self.curve_PIDs = [self.right_bottom_circle_PID, self.merging_circle_PID, self.left_center_circle_PID, self.left_top_circle_PID, self.right_top_circle_PID]
            self.dist = [self.merge_path_dist, self.main2_dist, self.return_first_dist, self.return_second_dist, self.return_third_dist]


    def generate_line(self, x1, y1, x2, y2):
        A = -(y2 - y1)
        B = -(x1 - x2)
        C = -(y1 * (x2 - x1) - (y2 - y1) * x1)
        return A, B, C

    def calc_distance(self, x1, y1, x2, y2):
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return distance

    def pid_lateral_controller(self, lateral_error, e_prev, e_int):
        e_int += lateral_error * self.delta_t
        e_der = (lateral_error - e_prev) / self.delta_t
        steering_angle = self.kp * lateral_error + self.ki * e_int + self.kd * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def run(self):
        self.kp, self.ki, self.kd = self.PIDs[self.current]
        self.current_line = self.lines[self.current]
        self.current_end_pt = self.points[self.next]

        #if the cav is near a critical point (which are turning corners), set path to a circle, change starting point and PID values to fit
        if abs(self.position_x  - self.current_end_pt[0])  < self.ranges[self.next][0] and \
            abs(self.position_z - self.current_end_pt[1]) < self.ranges[self.next][1] and \
            self.current_end_pt[0] != self.merging_pt_x and self.current_end_pt[1] != self.merging_pt_y:
            self.within_critical_range = True
            self.line_changed = False
            self.kp, self.ki, self.kd = self.curve_PIDs[self.next]
            lateral_error = (((self.position_x - self.circles[self.next][0])**2 + (self.position_z - self.circles[self.next][1])**2)**0.5 - self.circles[self.next][2])
            #print(self.ID, "in corner", lateral_error)

        #if a merging cav is near the merging point, switch to main path
        elif abs(self.position_x  - self.current_end_pt[0])  < self.ranges[self.next][0] and \
            abs(self.position_z - self.current_end_pt[1]) < self.ranges[self.next][1] and\
            self.current_end_pt[0] == self.merging_pt_x and self.current_end_pt[1] == self.merging_pt_y:

            self.within_critical_range = True
            self.line_changed = False
            self.current_line = self.lines[self.next]
            self.kp, self.ki, self.kd = self.PIDs[self.next]
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)
            #print(self.ID, "merging", lateral_error)

        #when the cav is on a straight path
        else:
            self.within_critical_range = False
            self.current_line = self.lines[self.current]
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)
            #print(self.ID, "out of corner", lateral_error)

        #once out of the turning point, follow the next line
        if not self.line_changed and not self.within_critical_range:
            self.current = (self.current+1) % len(self.lines)
            self.next = (self.next+1) % len(self.lines)
            self.line_changed = True
            self.within_critical_range = False
            self.current_line = self.lines[self.current]
            self.current_end_pt = self.points[self.next]
            self.kp, self.ki, self.kd = self.PIDs[self.current]
            self.e_prev_lateral= 0
            self.e_int_lateral = 0
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)
        #calculate steering and publisher to the listener node on the limo
        actual_velocity = self.velocity

        if self.ID == "limo770" or self.ID == "limo795":
            desired_velocity = actual_velocity + self.qp_solution.u* 0.1 # Use control input from QP solution
            #print("act vel", actual_velocity, "desired_velocity", desired_velocity,"qp_solutn", self.qp_solution.u)
        else:
            desired_velocity = 0.5

        steering_angle, self.e_prev_lateral, self.e_int_lateral = self.pid_lateral_controller(lateral_error, self.e_prev_lateral, self.e_int_lateral)
        control_input = -1
        desired_velocity = min(0.8, max(0, desired_velocity))
        #print("lateral_error of", self.ID, lateral_error)
        #rospy.loginfo(f"CAV{self.ID} Control Info - Steering Angle: {steering_angle}, Desired Velocity: {desired_velocity}, Control Input: {control_input}")

        # Publish control info
        control_info = ControlInfo()
        control_info.steering_angle = steering_angle
        control_info.desired_velocity = desired_velocity
        control_info.control_input = control_input
        self.control_info_pub.publish(control_info)