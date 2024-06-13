#!/usr/bin/env python3
# coding=UTF-8
import math
import numpy as np
from cvxopt.solvers import qp
# from pylimo import limo
import rospy
# import re
# import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import odeint
from cvxopt import matrix,solvers

class CAV():
    def __init__(self, node_name):
        self.node_name = node_name
        self.position_z = 0
        self.position_x = 0
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.position_ip_z = 0
        self.position_ip_x = 0
        self.ip_velocity = 0
        self.ip_acceleration = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.v_min = 0
        self.v_max = 0.6
        self.u_min = -2
        self.u_max = 2
        self.Delta_T = 0.05  # Control update period
        self.Lc = 0.0  # Center of the lane
        self.phiRearEnd = 50
        self.phiLateral = 50
        self.deltaSafetyDistance = 500
        self.max_steering_angle= 7000
        # Construct publisher
        rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_'+self.node_name, AckermannDrive, queue_size=10)
        rospy.Rate(10)

    def callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_yaw = 0
        self.Receivedata = 1

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
        distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        return distance

    def OCBF_SecondOrderDynamics(self, matrix_const, state):
        ocpar = [-0.593787660013256, 1.41421356237309, 0, 0, 2.38168230431317, 1.68410370801184]
        c = np.array(ocpar)
        x0 = np.array(state)
        eps = 10
        psc = 0.1
        t = 0.1

        vd = 1
        u_ref = c[0] * t + c[1]
        u_ref = 1

        b_vmax = self.v_max - x0[1]
        b_vmin = x0[1] - self.v_min

        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        def solveQP():
            A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0]])
            b = np.array([self.u_max, -self.u_min, phi0, b_vmax])

            if matrix_const[0][0] != -1:
                xip = matrix_const[0][1]
                h = xip - x0[2] - self.phiRearEnd * x0[1] - self.deltaSafetyDistance
                vip = matrix_const[0][2]
                LgB = self.phiRearEnd
                LfB = vip - x0[1]
                if LgB != 0:
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB + h])

            for row_index, row in enumerate(matrix_const):
                if row_index == 0:
                    continue
                if -1 in row:
                    continue
                else:
                    d1 = matrix_const[row_index][3]
                    d2 = state[row_index + 2]
                L = state[row_index + 2] + state[0]

                v0 = matrix_const[row_index][2]
                bigPhi = self.phiLateral * x0[0] / L
                h = d2 - d1 - bigPhi * x0[1] - self.deltaSafetyDistance

                uminValue = abs(self.u_min)
                hf = d2 - d1 - 0.5 * (v0 - x0[1]) ** 2 / uminValue - self.phiLateral * v0 * (
                        x0[0] + 0.5 * (x0[1] ** 2 - v0 ** 2) / uminValue) / L

                LgB = bigPhi
                LfB = v0 - x0[1] - self.phiLateral * x0[1] ** 2 / L
                if LgB != 0:
                    stop = 1
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB + h])

            H = matrix([[1, 0], [0, psc]])
            f = matrix([[-u_ref], [0]]).trans()
            H = matrix(H, tc='d')
            f = matrix(f, tc='d')
            A = matrix(A, tc='d')
            b = matrix(b, tc='d')


            solvers.options['show_progress'] = False
            Solution = qp(H, f, A, b)
            if Solution['status'] == 'optimal':
                print("QP Solved: Feasible")
            else:
                print("QP is not Feasible")
            return Solution['x'].trans()

        return solveQP()[0]

    def update_state(self, state, u, steering_angle):
        t_span = [0, self.Delta_T]
        solution = odeint(self.second_order_model, state, t_span, args=(u, steering_angle))
        return solution[-1]

    def second_order_model(self, x, t, u, steering_angle):
        dx = np.zeros(3)
        dx[0] = x[1]
        dx[1] = u
        dx[2] = steering_angle
        return dx

    def PID_controller(self, v_desired, v_current, e_prev, e_int):
        Kp = 0.05
        Ki = 0.07
        Kd = 0.05
        error = v_desired - v_current
        e_int += error * self.Delta_T
        e_der = (error - e_prev) / self.Delta_T
        u = Kp * error + Ki * e_int + Kd * e_der
        return u, error, e_int

    def PID_lateral_controller(self, lateral_error, e_prev, e_int):
        Kp = 0.000000005
        Ki = 0.000000
        Kd = 0.0000005
        e_int += lateral_error * self.Delta_T
        e_der = (lateral_error - e_prev) / self.Delta_T
        steering_angle = Kp * lateral_error + Ki * e_int + Kd * e_der
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        return steering_angle, lateral_error, e_int

    def detect_collision_threat(self, other_CAV):
        distance = abs(self.position_z - other_CAV.position_z)
        #distance = self.calc_distance(self.position_x, self.position_z, other_CAV.position_x, other_CAV.position_z)
        if distance < self.deltaSafetyDistance:
            return True
        return False

    def normal_acceleration(self):
        return 0.6
    def calc_distance(self, x1, y1, x2, y2):
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return distance

    def generate_line(self, x_1, y_1, x_2, y_2):
        A = -(y_2 - y_1)
        B = -(x_1 - x_2)
        C = -(y_1 * (x_2 - x_1) - (y_2 - y_1) * x_1)
        return A, B, C

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
        self.left_center_activation_range = (self.lane_width / 1.5, self.lane_width * 1)
        self.merging_pt_activation_range = (self.lane_width * 1, self.lane_width / 2)

        self.main_path = self.generate_line(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.return_first = self.generate_line(self.left_center_x, self.left_center_y, self.left_top_x, self.left_top_y)
        self.return_second = self.generate_line(self.left_top_x, self.left_top_y, self.right_top_x, self.right_top_y)
        self.return_third = self.generate_line(self.right_top_x, self.right_top_y, self.right_center_x, self.right_center_y)
        self.merging_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)
        self.off_path = self.generate_line(self.right_top_x, self.right_top_y, self.right_bottom_x, self.right_bottom_y)

        self.right_top_circle = (self.right_top_x - self.lane_width, self.right_top_y - self.lane_width, self.lane_width / 1.5)
        self.right_center_circle = (self.right_center_x + self.lane_width, self.right_center_y - self.lane_width, self.lane_width / 1.5)
        self.right_bottom_circle = (self.right_bottom_x + self.lane_width, self.right_bottom_y - self.lane_width, self.lane_width / 2)
        self.left_top_circle = (self.left_top_x - self.lane_width, self.left_top_y + self.lane_width, self.lane_width / 1.5)
        self.left_center_circle = (self.left_center_x + self.lane_width, self.left_center_y + self.lane_width, self.lane_width / 1.5)
        self.merging_circle = (self.merging_pt_x + self.lane_width, self.merging_pt_y - self.lane_width, self.lane_width)

        self.merging_path_dist = self.calc_distance(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)
        self.main_path_dist = self.calc_distance(self.merging_pt_x, self.merging_pt_y, self.left_center_x, self.left_center_y)
        self.half_main_path_dist = self.calc_distance(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.return_first_dist = self.calc_distance(self.left_center_x, self.left_center_y, self.left_top_x, self.left_top_y)
        self.return_second_dist = self.calc_distance(self.left_top_x, self.left_top_y, self.right_top_x, self.right_top_y)
        self.return_third_dist = self.calc_distance(self.right_top_x, self.right_top_y, self.right_center_x, self.right_center_y)
        self.off_path_dist = self.calc_distance(self.right_top_x, self.right_top_y, self.right_bottom_x, self.right_bottom_y)

        self.merge_path_PID = (-0.0005, -0.00005, -0.001)
        self.main_path_PID = (-0.0005, -0.00005, -0.001)
        self.return_first_PID = (-0.0005, -0.00005, -0.001)
        self.return_second_PID = (-0.0004, -0.00005, -0.001)
        self.return_third_PID = (-0.0005, -0.00005, -0.001)

        self.right_bottom_circle_PID = (-0.50, -0.00045, -0.037)
        self.right_center_circle_PID = (-0.0030, -0.000045, -0.0017)
        self.left_center_circle_PID = (-0.56, -0.00045, -0.037)
        self.left_top_circle_PID = (-0.60, -0.045, -0.050)
        self.right_top_circle_PID = (-0.55, -0.00045, -0.037)
        self.merging_circle_PID = (-0.0005, -0.00045, -0.003)

        if isMain:
            self.points = [(self.right_center_x, self.right_center_y), (self.left_center_x, self.left_center_y),
                           (self.left_top_x, self.left_top_y), (self.right_top_x, self.right_top_y)]
            self.lines = [self.main_path, self.return_first, self.return_second, self.return_third]
            self.dists = [self.main_path_dist, self.return_first_dist, self.return_second_dist, self.return_third_dist]
            self.ranges = [self.right_center_activation_range, self.left_center_activation_range,
                           self.left_top_activation_range, self.right_top_activation_range]
            self.circles = [self.right_center_circle, self.left_center_circle, self.left_top_circle, self.right_top_circle]
            self.PIDs = [self.main_path_PID, self.return_first_PID, self.return_second_PID, self.return_third_PID]
            self.curve_PIDs = [self.right_center_circle_PID, self.left_center_circle_PID, self.left_top_circle_PID, self.right_top_circle_PID]

        else:
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y), (self.left_center_x, self.left_center_y),
                           (self.left_top_x, self.left_top_y), (self.right_top_x, self.right_top_y)]
            self.lines = [self.merging_path, self.main_path, self.return_first, self.return_second, self.off_path]
            self.dists = [self.merging_path_dist, self.half_main_path_dist, self.return_first_dist, self.return_second_dist, self.return_third_dist]
            self.PIDs = [self.merge_path_PID, self.main_path_PID, self.return_first_PID, self.return_second_PID, self.return_third_PID]
            self.ranges = [self.right_bottom_activation_range, self.merging_pt_activation_range,
                           self.left_center_activation_range, self.left_top_activation_range, self.right_top_activation_range]
            self.circles = [self.right_bottom_circle, self.merging_circle, self.left_center_circle, self.left_top_circle, self.right_top_circle]
            self.curve_PIDs = [self.right_bottom_circle_PID, self.merging_circle_PID, self.left_center_circle_PID, self.left_top_circle_PID, self.right_top_circle_PID]


    def generate_line(self, x_1, y_1, x_2, y_2):
        A = -(y_2 - y_1)
        B = -(x_1 - x_2)
        C = -(y_1 * (x_2 - x_1) - (y_2 - y_1) * x_1)
        return A, B, C

    def detect_collision_threat(self, other_cav):
        distance = self.calc_distance(self.position_x, self.position_z, other_cav.position_x, other_cav.position_z)
        return distance < self.deltaSafetyDistance

    def normal_acceleration(self):
        return 0.2

    def run (self):
        message = "%s %s %s %s" %(self.position_z, self.position_x, self.velocity, self.acceleration)
        self.pub.publish(message)

    def callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_yaw = 0
        self.Receivedata = 1

    def steeringAngleToSteeringCommand(self, refAngle):
        x = refAngle
        y = 0.7 * x
        return y

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    def PIDController(self, x, x_ref, prev_e, prev_int, delta_t, Kp, Ki, Kd):
        e = x_ref - x
        e_int = prev_int + e * delta_t
        e_int = max(min(e_int, 0.5), -0.5)
        e_der = (e - prev_e) / delta_t

        u_k = Kp * e
        u_i = Ki * e_int
        u_d = Kd * e_der
        u = Kp * e + Ki * e_int + Kd * e_der

        return u, u_k, u_i, u_d, e, e_int

    def PIDController(self, e, prev_e, prev_int, delta_t, Kp, Ki, Kd):
        if e <= 1 and e >= -1:
            e_int = 0
        e_int = prev_int + e * delta_t
        e_int = max(min(e_int, 0.3), -0.3)
        e_der = (e - prev_e) / delta_t

        u_k = Kp * e
        u_i = Ki * e_int
        u_d = Kd * e_der
        u = Kp * e + Ki * e_int + Kd * e_der

        return u, u_k, u_i, u_d, e, e_int

    def control(self, e, v_ref, eprev_lateral, eint_lateral, dt):
        if eprev_lateral * e <= 0:
            eint_lateral = 0
        kp = self.kp
        ki = self.ki
        kd = self.kd

        ref_steer, u_k, u_i, u_d, eprev_lateral, eint_lateral = self.PIDController(e, eprev_lateral, eint_lateral, dt, kp, ki, kd)

        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = self.steeringAngleToSteeringCommand(ref_steer)
        return eprev_lateral, eint_lateral, drive_msg
