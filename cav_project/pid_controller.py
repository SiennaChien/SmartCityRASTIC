#!/usr/bin/env python3
import rospy
from cav_project.msg import limo_info, QP_solution
from ackermann_msgs.msg import AckermannDrive

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller')
        self.cav_info_sub_cav1 = rospy.Subscriber('/cav_info_cav1', limo_info, self.cav_info_callback_cav1)
        self.cav_info_sub_cav2 = rospy.Subscriber('/cav_info_cav2', limo_info, self.cav_info_callback_cav2)
        self.qp_solution_sub_cav1 = rospy.Subscriber('/qp_solution_cav1', QP_solution, self.qp_solution_callback_cav1)
        self.qp_solution_sub_cav2 = rospy.Subscriber('/qp_solution_cav2', QP_solution, self.qp_solution_callback_cav2)
        self.drive_pub_cav1 = rospy.Publisher('vel_steer_cav1', AckermannDrive, queue_size=10)
        self.drive_pub_cav2 = rospy.Publisher('vel_steer_cav2', AckermannDrive, queue_size=10)
        self.cav_info_cav1 = None
        self.cav_info_cav2 = None
        self.qp_solution_cav1 = None
        self.qp_solution_cav2 = None
        self.kp_lateral = 0.0015
        self.ki_lateral = 0.000045
        self.kd_lateral = 0.0017
        self.kp_longitudinal = 0.1
        self.ki_longitudinal = 0.01
        self.kd_longitudinal = 0.05
        self.rate = rospy.Rate(20)

    def cav_info_callback_cav1(self, msg):
        self.cav_info_cav1 = msg

    def cav_info_callback_cav2(self, msg):
        self.cav_info_cav2 = msg

    def qp_solution_callback_cav1(self, msg):
        self.qp_solution_cav1 = msg

    def qp_solution_callback_cav2(self, msg):
        self.qp_solution_cav2 = msg

    def pid_lateral_controller(self, lateral_error, e_prev, e_int, delta_t):
        e_int += lateral_error * delta_t
        e_der = (lateral_error - e_prev) / delta_t
        steering_angle = self.kp_lateral * lateral_error + self.ki_lateral * e_int + self.kd_lateral * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def pid_longitudinal_controller(self, desired_velocity, actual_velocity, e_prev, e_int, delta_t):
        error = desired_velocity - actual_velocity
        e_int += error * delta_t
        e_der = (error - e_prev) / delta_t
        control_input = self.kp_longitudinal * error + self.ki_longitudinal * e_int + self.kd_longitudinal * e_der
        control_input = max(min(control_input, 1), -1)
        return control_input, error, e_int

    def run(self):
        e_prev_lateral_cav1 = 0
        e_int_lateral_cav1 = 0
        e_prev_lateral_cav2 = 0
        e_int_lateral_cav2 = 0
        e_prev_longitudinal_cav1 = 0
        e_int_longitudinal_cav1 = 0
        e_prev_longitudinal_cav2 = 0
        e_int_longitudinal_cav2 = 0
        delta_t = 0.05

        while not rospy.is_shutdown():
            if self.cav_info_cav1 and self.qp_solution_cav1:
                lateral_error_cav1 = -(self.cav_info_cav1.d1.data * self.cav_info_cav1.d2.data + self.cav_info_cav1.d2.data * self.cav_info_cav1.d2.data + self.cav_info_cav1.d2.data) / ((self.cav_info_cav1.d1.data**2 + self.cav_info_cav1.d2.data**2)**0.5)
                steering_angle_cav1, e_prev_lateral_cav1, e_int_lateral_cav1 = self.pid_lateral_controller(lateral_error_cav1, e_prev_lateral_cav1, e_int_lateral_cav1, delta_t)
                desired_velocity_cav1 = self.cav_info_cav1.vel.data + self.qp_solution_cav1.u.data * 0.05
                actual_velocity_cav1 = self.cav_info_cav1.vel.data
                control_input_cav1, e_prev_longitudinal_cav1, e_int_longitudinal_cav1 = self.pid_longitudinal_controller(desired_velocity_cav1, actual_velocity_cav1, e_prev_longitudinal_cav1, e_int_longitudinal_cav1, delta_t)
                drive_msg_cav1 = AckermannDrive()
                drive_msg_cav1.speed = control_input_cav1
                drive_msg_cav1.steering_angle = steering_angle_cav1
                self.drive_pub_cav1.publish(drive_msg_cav1)

            if self.cav_info_cav2 and self.qp_solution_cav2:
                lateral_error_cav2 = -(self.cav_info_cav2.d1.data * self.cav_info_cav2.d2.data + self.cav_info_cav2.d2.data * self.cav_info_cav2.d2.data + self.cav_info_cav2.d2.data) / ((self.cav_info_cav2.d1.data**2 + self.cav_info_cav2.d2.data**2)**0.5)
                steering_angle_cav2, e_prev_lateral_cav2, e_int_lateral_cav2 = self.pid_lateral_controller(lateral_error_cav2, e_prev_lateral_cav2, e_int_lateral_cav2, delta_t)
                desired_velocity_cav2 = self.cav_info_cav2.vel.data + self.qp_solution_cav2.u.data * 0.05
                actual_velocity_cav2 = self.cav_info_cav2.vel.data
                control_input_cav2, e_prev_longitudinal_cav2, e_int_longitudinal_cav2 = self.pid_longitudinal_controller(desired_velocity_cav2, actual_velocity_cav2, e_prev_longitudinal_cav2, e_int_longitudinal_cav2, delta_t)
                drive_msg_cav2 = AckermannDrive()
                drive_msg_cav2.speed = control_input_cav2
                drive_msg_cav2.steering_angle = steering_angle_cav2
                self.drive_pub_cav2.publish(drive_msg_cav2)

            self.rate.sleep()

if __name__ == '__main__':
    controller = PIDController()
    controller.run()
