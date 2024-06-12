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
        self.limo_array_data.limo_infos = [self.limo_data_cav1, self.limo_data_cav2]
        # Uncomment if you need to publish the array data somewhere
        # self.limo_array_pub.publish(self.limo_array_data)

        self.qp_solution_cav1_sub = rospy.Subscriber('/qp_solution_cav1', QP_solution, self.qp_solution_cav1_callback)
        self.qp_solution_cav2_sub = rospy.Subscriber('/qp_solution_cav2', QP_solution, self.qp_solution_cav2_callback)
        self.cav_info_cav1_sub = rospy.Subscriber('/limo_info_cav1', limo_info, self.cav_info_cav1_callback)
        self.cav_info_cav2_sub = rospy.Subscriber('/limo_info_cav2', limo_info, self.cav_info_cav2_callback)
        self.limo_array_pub = rospy.Publisher('/limo_info_array', limo_info_array, queue_size=10)

        self.mocap_sub_cav1 = rospy.Subscriber('/vrpn_client_node/limo770/pose', PoseStamped, self.mocap_callback_cav1)
        self.mocap_sub_cav2 = rospy.Subscriber('/vrpn_client_node/limo155/pose', PoseStamped, self.mocap_callback_cav2)
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

        self.mocap_position_cav1_x = 0
        self.mocap_position_cav1_z = 0
        self.mocap_position_cav2_x = 0
        self.mocap_position_cav2_z = 0


    def mocap_callback_cav1(self, msg):
        self.mocap_position_cav1_z = msg.pose.position.z * 1000
        self.mocap_position_cav1_x = msg.pose.position.x * 1000
        self.position_yaw = 0
        self.Receivedata = 1
        #self.publish_info()

    def mocap_callback_cav2(self, msg):
        self.mocap_position_cav2_z = msg.pose.position.z * 1000
        self.mocap_position_cav2_x = msg.pose.position.x * 1000
        self.position_yaw = 0
        self.Receivedata = 1
        #self.publish_info()

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
        self.merge_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)

        self.merge_path_PID = (-0.0005, -0.00005, -0.001)
        self.main_path_PID = (-0.0005, -0.00005, -0.001)

        if isMain:
            self.points = [(self.right_center_x, self.right_center_y), (self.left_center_x, self.left_center_y)]
            self.lines = [self.main_path]
            self.dists = [self.calc_distance(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)]
            self.PIDs = [self.main_path_PID]
        else:
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y)]
            self.lines = [self.merge_path]
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

    def run(self):
        while not rospy.is_shutdown():
            self.generate_map(True)
            if True: #self.qp_solution_cav1 and self.qp_solution_cav2 and self.cav_info_cav1 and self.cav_info_cav2:
                # Calculate desired velocities using QP solutions
                desired_velocity_cav1 = self.cav_info_cav1.vel.data + self.qp_solution_cav1.u * 0.05
                desired_velocity_cav2 = self.cav_info_cav2.vel.data + self.qp_solution_cav2.u * 0.05

                # PID Lateral Control for CAV1
                if self.cav_info_cav1.d1.data**2 + self.cav_info_cav1.d2.data**2 != 0:
                    lateral_error_cav1 = -(self.main_path[0]*self.mocap_position_cav1_x + self.main_path[1]*self.mocap_position_cav1_z + self.main_path[2])/((self.main_path[0]**2+self.main_path[1]**2)**0.5)
                else:
                    lateral_error_cav1 = -(self.main_path[0]*self.mocap_position_cav1_x + self.main_path[1]*self.mocap_position_cav1_z + self.main_path[2])/((self.main_path[0]**2+self.main_path[1]**2)**0.5)
                steering_angle_cav1, self.e_prev_lateral_cav1, self.e_int_lateral_cav1 = self.pid_lateral_controller(lateral_error_cav1, self.e_prev_lateral_cav1, self.e_int_lateral_cav1)

                # PID Longitudinal Control for CAV1
                actual_velocity_cav1 = self.cav_info_cav1.vel.data
                control_input_cav1, self.e_prev_longitudinal_cav1, self.e_int_longitudinal_cav1 = self.pid_longitudinal_controller(desired_velocity_cav1, actual_velocity_cav1, self.e_prev_longitudinal_cav1, self.e_int_longitudinal_cav1)

 #Print control info for CAV1
                rospy.loginfo(f"CAV1 Control Info - Steering Angle: {steering_angle_cav1}, Desired Velocity: {desired_velocity_cav1}, Control Input: {control_input_cav1}")

                # Publish control info for CAV1
                control_info_cav1 = ControlInfo()
                control_info_cav1.steering_angle = steering_angle_cav1
                control_info_cav1.desired_velocity = desired_velocity_cav1
                control_info_cav1.control_input = control_input_cav1
                self.control_info_pub_cav1.publish(control_info_cav1)

                # PID Lateral Control for CAV2
                if self.cav_info_cav2.d1.data**2 + self.cav_info_cav2.d2.data**2 != 0:
                    lateral_error_cav2 = -(self.merge_path[0]*self.mocap_position_cav2_x + self.merge_path[1]*self.mocap_position_cav2_z + self.merge_path[2])/((self.merge_path[0]**2+self.merge_path[1]**2)**0.5)
                else:
                    lateral_error_cav2 = -(self.merge_path[0]*self.mocap_position_cav2_x + self.merge_path[1]*self.mocap_position_cav2_z + self.merge_path[2])/((self.merge_path[0]**2+self.merge_path[1]**2)**0.5)
                steering_angle_cav2, self.e_prev_lateral_cav2, self.e_int_lateral_cav2 = self.pid_lateral_controller(lateral_error_cav2, self.e_prev_lateral_cav2, self.e_int_lateral_cav2)

                # PID Longitudinal Control for CAV2
                actual_velocity_cav2 = self.cav_info_cav2.vel.data
                control_input_cav2, self.e_prev_longitudinal_cav2, self.e_int_longitudinal_cav2 = self.pid_longitudinal_controller(desired_velocity_cav2, actual_velocity_cav2, self.e_prev_longitudinal_cav2, self.e_int_longitudinal_cav2)
 #Print control info for CAV1
                rospy.loginfo(f"CAV2 Control Info - Steering Angle: {steering_angle_cav2}, Desired Velocity: {desired_velocity_cav2}, Control Input: {control_input_cav2}")

                # Publish control info for CAV2


                print("lateral error cav1", lateral_error_cav1, "lateral error cav2", lateral_error_cav2)

                control_info_cav2 = ControlInfo()
                control_info_cav2.steering_angle = steering_angle_cav2
                control_info_cav2.desired_velocity = desired_velocity_cav2
                control_info_cav2.control_input = control_input_cav2
                self.control_info_pub_cav2.publish(control_info_cav2)

            self.rate.sleep()

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()
