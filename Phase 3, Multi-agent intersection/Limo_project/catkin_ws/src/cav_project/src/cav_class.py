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
        self.current_collision = 0
        self.next_collision = 1
        self.current_line = self.lines[self.current]
        self.current_end_pt = self.points[self.next]
        self.current_collision_pt1 = self.collision_pts[self.current_collision]
        self.current_collision_pt2 = self.collision_pts[self.next_collision]

    def mocap_callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_yaw = 0
        self.Receivedata = 1
        self.current_position = (self.position_x, self.position_z)

    def qp_solution_callback(self, msg):
        self.qp_solution = msg

    def cav_info_callback(self, msg):
        self.cav_info = msg
        self.velocity = self.cav_info.vel.data

    def generate_map(self, isMain, enter):
        #coordinates of every critical point
        self.lane_width = 450
        self.pt_a = (2933, -1980)
        self.pt_b = (3044, 2536)
        self.pt_c = (1922, -1977)
        self.pt_d = (1825, -652)
        self.pt_e = (2040, 2519)
        self.pt_f = (683, 2536) 
        self.pt_g = (29, 222)
        self.pt_h = (394, 554)
        self.pt_i = (1560, -1945)
        self.pt_j = (-2070, -1992)
        self.pt_k = (-2007, 243)
        self.pt_l = (-2028, 639)
        self.pt_m = (-1954, 2640)
        self.pt_n = (-2439, -1980)
        self.pt_o = (-2460, 253)
        self.pt_p = (-2454, 628)
        self.pt_q = (-2356, 2654)
        self.pt_r = (-4683, -1980)
        self.pt_s = (-4683, 259)
        self.pt_t = (-4659, 682)
        self.pt_u = (-4660, 2654)

        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.path_A = self.generate_line(self.pt_a, self.pt_r)
        self.path_B = self.generate_line(self.pt_a, self.pt_b)
        self.path_C = self.generate_line(self.pt_b, self.pt_u)
        self.path_D = self.generate_line(self.pt_c, self.pt_e)
        self.path_E = self.generate_line(self.pt_d, self.pt_f)
        self.path_F = self.generate_line(self.pt_i, self.pt_h)
        self.path_G = self.generate_line(self.pt_j, self.pt_m)
        self.path_H = self.generate_line(self.pt_n, self.pt_q)
        self.path_I = self.generate_line(self.pt_g, self.pt_s)
        self.path_J = self.generate_line(self.pt_h, self.pt_t)
        self.path_K = self.generate_line(self.pt_r, self.pt_u)

        #data points that characterize each circle for the corners - center x, center y, radius. Each variable is a tuple (A, B, C)
        self.circle_a = (self.pt_a[0] - self.lane_width, self.pt_a[1] + self.lane_width, self.lane_width/1.3)
        self.circle_b = (self.pt_b[0] - self.lane_width, self.pt_b[1] - self.lane_width, self.lane_width/1.5)
        self.circle_c = (self.pt_c[0] + self.lane_width, self.pt_c[1] + self.lane_width, self.lane_width/1.5)
        self.circle_d = (self.pt_d[0] + self.lane_width, self.pt_d[1] - self.lane_width, self.lane_width) #in practice this is not use
        self.circle_e = (self.pt_e[0] + self.lane_width, self.pt_e[1] - self.lane_width, self.lane_width/1.5)
        self.circle_f = (self.pt_f[0] + self.lane_width, self.pt_f[1] - self.lane_width, self.lane_width/2)
        self.circle_g = (self.pt_d[0] - self.lane_width, self.pt_d[1] - self.lane_width, self.lane_width) 
        self.circle_h = (self.pt_h[0] - self.lane_width, self.pt_h[1] - self.lane_width, self.lane_width) 
        self.circle_i = (self.pt_i[0] + self.lane_width, self.pt_i[1] + self.lane_width, self.lane_width) 
        self.circle_j = (self.pt_j[0] + self.lane_width, self.pt_j[1] + self.lane_width, self.lane_width) 
        self.circle_k = (self.pt_k[0] + self.lane_width, self.pt_k[1] - self.lane_width, self.lane_width) 
        self.circle_l = (self.pt_l[0] + self.lane_width, self.pt_l[1] + self.lane_width, self.lane_width) 
        self.circle_m = (self.pt_m[0] + self.lane_width, self.pt_m[1] - self.lane_width, self.lane_width) 
        self.circle_n = (self.pt_n[0] - self.lane_width, self.pt_n[1] + self.lane_width, self.lane_width) 
        self.circle_o = (self.pt_o[0] - self.lane_width, self.pt_o[1] - self.lane_width, self.lane_width) 
        self.circle_p = (self.pt_p[0] - self.lane_width, self.pt_p[1] + self.lane_width, self.lane_width) 
        self.circle_q = (self.pt_q[0] - self.lane_width, self.pt_q[1] - self.lane_width, self.lane_width) 
        self.circle_r = (self.pt_r[0] + self.lane_width, self.pt_r[1] + self.lane_width, self.lane_width) 
        self.circle_s = (self.pt_s[0] + self.lane_width, self.pt_s[1] - self.lane_width, self.lane_width) 
        self.circle_t = (self.pt_t[0] + self.lane_width, self.pt_t[1] + self.lane_width, self.lane_width) 
        self.circle_u = (self.pt_u[0] + self.lane_width, self.pt_u[1] - self.lane_width, self.lane_width) 
        
        #the ranges near each corner that activates the circle path for the limo to follow
        self.act_range_a = (self.lane_width * 1, self.lane_width / 1.7)
        self.act_range_b = (self.lane_width * 0.8, self.lane_width * 1)
        self.act_range_c = (self.lane_width / 1, self.lane_width * 1)
        self.act_range_d = (self.lane_width * 0.7, self.lane_width * 0.5)
        self.act_range_e = (self.lane_width * 1, self.lane_width * 0.8)
        self.act_range_f = (self.lane_width * 1, self.lane_width*1.2)
        self.act_range_g = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_h = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_i = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_j = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_k = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_l = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_m = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_n = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_o = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_p = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_q = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_r = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_s = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_t = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_u = (self.lane_width * 1, self.lane_width * 1)

        #PID values of each line, each element is a tuple (kp, ki, kd)
        self.path_A_PID = (0.002, 0.00005, 0.003)
        self.path_B_PID = (0.0008, 0.00008, 0.003)
        self.path_C_PID = (0.0008, 0.00008, 0.005)
        self.path_D_PID = (0.0002, 0.00005, 0.001)
        self.path_D2_PID = (0.0007, 0.00008, 0.001)
        self.path_E_PID = (0.0005, 0.00003, 0.003)
        self.path_F_PID = (0.0008, 0.00008, 0.003)
        self.path_G_PID = (0.0003, 0.00004, 0.001)
        self.path_H_PID = (0.0008, 0.00008, 0.003)
        self.path_I_PID = (0.0008, 0.00008, 0.003)
        self.path_J_PID = (0.0008, 0.00008, 0.003)
        self.path_K_PID = (0.0008, 0.00008, 0.003)

        #PID values of each circle, each element is a tuple (kp, ki, kd)
        self.circle_a_PID = (-0.7, -0.00045, -0.050)
        self.circle_b_PID = (-0.55, -0.00045, -0.037)
        self.circle_c_PID = (-0.56, -0.00045, -0.037)
        self.circle_d_PID = (-0.0005, -0.00045, -0.003)
        self.circle_e_PID = (-0.0030, -0.000045, -0.0017)
        self.circle_f_PID = (-0.050, -0.00045, -0.037)
        self.circle_g_PID = (-0.050, -0.00045, -0.037)
        self.circle_h_PID = (-0.050, -0.00045, -0.037)
        self.circle_i_PID = (-0.050, -0.00045, -0.037)
        self.circle_j_PID = (-0.050, -0.00045, -0.037)
        self.circle_k_PID = (-0.050, -0.00045, -0.037)
        self.circle_l_PID = (-0.050, -0.00045, -0.037)
        self.circle_m_PID = (-0.050, -0.00045, -0.037)
        self.circle_n_PID = (-0.050, -0.00045, -0.037)
        self.circle_o_PID = (-0.050, -0.00045, -0.037)
        self.circle_p_PID = (-0.050, -0.00045, -0.037)
        self.circle_q_PID = (-0.050, -0.00045, -0.037)
        self.circle_r_PID = (-0.050, -0.00045, -0.037)
        self.circle_s_PID = (-0.050, -0.00045, -0.037)
        self.circle_t_PID = (-0.050, -0.00045, -0.037)
        self.circle_u_PID = (-0.050, -0.00045, -0.037)

        if isMain: #if the limo runs along the main path
            #array to store all points at which the limo needs to turn, in order of traversal
            self.turning_pts = [self.pt_e, self.pt_c, self.pt_a, self.pt_b]
            #array to store all possible collision points, in order of traversal
            self.collision_pts = [self.pt_d]
            #array to store all points, turning and collision, in order of traversal
            self.all_pts = [self.pt_e, self.pt_d, self.pt_c, self.pt_a, self.pt_b]
            #array to store all lines, in order of traversal
            self.lines = [self.path_D, self.path_A, self.path_B, self.path_C]
            #the activation range of the corners, in order of traversal
            self.ranges = [self.act_range_e, self.act_range_c, self.act_range_a, self.act_range_b]
            #array to store the circles for the corners, in order of traversal
            self.circles = [self.circle_e, self.circle_c, self.circle_a, self.circle_b]
            #array to store PID values of each line, in order of traversal, each element is a tuple (kp, ki, kd)
            self.PIDs = [self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
            #array to store PID values of each circle, in order of traversal, each element is a tuple (kp, ki, kd)
            self.curve_PIDs = [self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID]
            #array to store all distances of paths, in order of traversal
            self.dist = self.calc_dist_array(self.all_pts)

        else: #if the limo runs along the merging path
            self.tuning_pts = [self.pt_f, self.pt_d, self.pt_c, self.pt_a, self.pt_b]
            self.collision_pts = [self.pt_d]
            self.all_pts = [self.pt_f, self.pt_d, self.pt_c, self.pt_a, self.pt_b]
            self.lines = [self.path_E, self.path_D, self.path_A, self.path_B, self.path_C]
            self.ranges = [self.act_range_f, self.act_range_d, self.act_range_c, self.act_range_a, self.act_range_b]
            self.circles = [self.circle_f, self.circle_d, self.circle_c, self.circle_a, self.circle_b]
            self.PIDs = [self.path_E_PID, self.path_D2_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
            self.curve_PIDs = [self.circle_f_PID, self.circle_d_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID]
            self.dist = self.calc_dist_array(self.all_pts)
        
        #the four paths below are for tuning the straight intersection paths
        if enter == 'H':
            self.lines = [self.path_H]
            self.collision_pts = [self.pt_o, self.pt_p]
            self.tuning_pts = [self.pt_n, self.pt_q]
            self.all_pts = [self.pt_n, self.pt_o, self.pt_p, self.pt_q]
            self.ranges = [self.act_range_n, self.act_range_q]
            self.circles = [self.circle_n,self.circle_q]
            self.PIDs = [self.path_H_PID]
            self.curve_PIDs = [self.circle_n_PID, self.circle_q_PID]
            self.dist = self.calc_dist_array(self.all_pts)
        
        # elif enter == 'G':
        #     self.lines = [self.path_G]
        #     self.points = [self.pt_m, self.pt_j]
        #     self.ranges = [self.act_range_m, self.act_range_j]
        #     self.circles = [self.circle_m, self.circle_j]
        #     self.PIDs = [self.path_G_PID]
        #     self.curve_PIDs = [self.circle_m_PID, self.circle_j_PID]
        #     self.dist = self.calc_dist_array(self.points)
        # elif enter == 'J':
        #     self.lines = [self.path_J]
        #     self.points = [self.pt_t, self.pt_h]
        #     self.ranges = [self.act_range_t, self.act_range_h]
        #     self.circles = [self.circle_t, self.circle_h]
        #     self.PIDs = [self.path_J_PID]
        #     self.curve_PIDs = [self.circle_t_PID, self.circle_h_PID]
        #     self.dist = self.calc_dist_array(self.points)
        # elif enter == 'I':
        #     self.lines = [self.path_I]
        #     self.points = [self.pt_g, self.pt_s]
        #     self.ranges = [self.act_range_g, self.act_range_s]
        #     self.circles = [self.circle_g, self.circle_s]
        #     self.PIDs = [self.path_I_PID]
        #     self.curve_PIDs = [self.circle_g_PID, self.circle_s_PID]
        #     self.dist = self.calc_dist_array(self.points)

        # #the 6 paths below collectively traverse every line and turn of the intersection map
        # elif enter == 't'and exit == 'h':
        #     self.lines = [self.path_C, self.path_K, self.path_J, self.path_F, self.path_A, self.path_B, self.path_C]
        #     self.points = [self.pt_f, self.pt_u, self.pt_t, self.pt_h, self.pt_i, self.pt_a, self.pt_b, self.pt_f]
        #     self.ranges = [self.act_range_f, self.act_range_u, self.act_range_t, self.act_range_h, self.act_range_i, self.act_range_a, self.act_range_b, self.act_range_f]
        #     self.circles = [self.circle_f, self.circle_u, self.circle_t, self.circle_h, self.circle_i, self.circle_a, self.circle_b, self.circle_f]
        #     self.PIDs = [self.path_C_PID, self.path_K_PID, self.path_J_PID, self.path_F_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
        #     self.curve_PIDs = [self.circle_f_PID, self.circle_u_PID, self.circle_t_PID, self.circle_h_PID, self.circle_i_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        #     self.dist = self.calc_dist_array(self.all_points)
        
        # elif enter == 'm'and exit == 'h':
        #     self.lines = [self.path_C, self.path_G, self.path_J, self.path_F, self.path_A, self.path_B, self.path_C]
        #     self.points = [self.pt_f, self.pt_m, self.pt_l, self.pt_h, self.pt_i, self.pt_a, self.pt_b, self.pt_f]
        #     self.ranges = [self.act_range_f, self.act_range_m, self.act_range_l, self.act_range_h, self.act_range_i, self.act_range_a, self.act_range_b, self.act_range_f]
        #     self.circles = [self.circle_f, self.circle_m, self.circle_l, self.circle_h, self.circle_i, self.circle_a, self.circle_b, self.circle_f]
        #     self.PIDs = [self.path_C_PID, self.path_G_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
        #     self.curve_PIDs = [self.circle_f_PID, self.circle_m_PID, self.circle_l_PID, self.circle_h_PID, self.circle_i_PID,  self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        #     self.dist = self.calc_dist_array(self.all_points)
        
        # elif enter == 'n'and exit == 'q':
        #     self.lines = [self.path_C, self.path_K, self.path_A, self.path_H, self.path_I, self.path_K, self.path_A, self.path_H, self.path_C, self.path_K, self.path_J, self.path_H, self.path_C, self.path_K, self.path_A, self.path_B, self.path_C]
        #     self.points = [self.pt_f, self.pt_u, self.pt_r, self.pt_n, self.pt_o, self.pt_s, self.pt_r, self.pt_n, self.pt_q, self.pt_u, self.pt_t, self.pt_p, self.pt_q, self.pt_u, self.pt_r, self.pt_a, self.pt_b, self.pt_f]
        #     self.ranges = [self.act_range_f, self.act_range_u, self.act_range_r, self.act_range_n, self.act_range_o, self.act_range_s, self.act_range_r, self.act_range_n, self.act_range_q, self.act_range_u, self.act_range_t, self.act_range_p, self.act_range_q, self.act_range_u, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_f]
        #     self.circles = [self.circle_f, self.circle_u, self.circle_r, self.circle_n, self.circle_o, self.circle_s, self.circle_r, self.circle_n, self.circle_q, self.circle_u, self.circle_t, self.circle_p, self.circle_q, self.circle_u, self.circle_r, self.circle_a, self.circle_b, self.circle_f]
        #     self.PIDs = [self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_H_PID, self.path_I_PID, self.path_K_PID, self.path_A_PID, self.path_H_PID, self.path_C_PID, self.path_K_PID, self.path_J_PID, self.path_H_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
        #     self.curve_PIDs = [self.circle_f_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID, self.circle_o_PID, self.circle_s_PID, self.circle_r_PID, self.circle_n_PID, self.circle_q_PID, self.circle_u_PID, self.circle_t_PID, self.circle_p_PID, self.circle_q_PID, self.circle_u_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        #     self.dist = self.calc_dist_array(self.all_points)
        
        # elif enter == 'm'and exit == 'j':
        #     self.lines = [self.path_C, self.path_G, self.path_A, self.path_B, self.path_C]
        #     self.points = [self.pt_f, self.pt_m, self.pt_j,self.pt_a, self.pt_b, self.pt_f]
        #     self.ranges = [self.act_range_f, self.act_range_m, self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_f]
        #     self.circles = [self.circle_f, self.circle_m, self.circle_j, self.circle_a, self.circle_b, self.circle_f]
        #     self.PIDs = [self.path_C_PID, self.path_G_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
        #     self.curve_PIDs = [self.circle_f_PID, self.circle_m_PID, self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        #     self.dist = self.calc_dist_array(self.all_points)
        
        # elif enter == 'g'and exit == 'j':
        #     self.lines = [self.path_I, self.path_G, self.path_A, self.path_B, self.path_C]
        #     self.points = [self.pt_g, self.pt_k, self.pt_j,self.pt_a, self.pt_b, self.pt_f]
        #     self.ranges = [self.act_range_g, self.act_range_k, self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_f]
        #     self.circles = [self.circle_g, self.circle_k, self.circle_j, self.circle_a, self.circle_b, self.circle_f]
        #     self.PIDs = [self.path_I_PID, self.path_G_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
        #     self.curve_PIDs = [self.circle_g_PID, self.circle_k_PID, self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        #     self.dist = self.calc_dist_array(self.all_points)
        
        # elif enter == 'g'and exit == 's':
        #     self.lines = [self.path_I, self.path_K, self.path_A, self.path_B, self.path_C]
        #     self.points = [self.pt_g, self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_f]
        #     self.ranges = [self.act_range_g, self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_f]
        #     self.circles = [self.circle_g, self.circle_s, self.circle_r, self.circle_a, self.circle_b, self.circle_f]
        #     self.PIDs = [self.path_I_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
        #     self.curve_PIDs = [self.circle_g_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID]
        #     self.dist = self.calc_dist_array(self.all_points)
        
    #helper functions for generate_map()
    def generate_line(self, pt_1, pt_2):
        A = -(pt_2[1] - pt_1[1])
        B = -(pt_1[0] - pt_2[0])
        C = -(pt_1[1] * (pt_2[0] - pt_1[0]) - (pt_2[1] - pt_1[1]) * pt_1[0])
        return A, B, C

    def calc_distance(self, pt_1, pt_2):
        distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
        return distance
    
    def calc_dist_array(self, points):
        dist = []
        for i in range(len(points)-1):
            dist.append(self.calc_distance(points[i], points[i+1]))
        return dist

    def pid_lateral_controller(self, lateral_error, e_prev, e_int):
        e_int += lateral_error * self.delta_t
        e_der = (lateral_error - e_prev) / self.delta_t
        steering_angle = self.kp * lateral_error + self.ki * e_int + self.kd * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def run(self):
        self.kp, self.ki, self.kd = self.PIDs[self.current]
        self.current_line = self.lines[self.current]
        self.current_end_pt = self.turning_pts[self.next]

        #if the cav is near a critical point (which are turning corners), set path to a circle, change starting point and PID values to fit
        if abs(self.position_x  - self.current_end_pt[0])  < self.ranges[self.next][0] and \
            abs(self.position_z - self.current_end_pt[1]) < self.ranges[self.next][1] and \
            self.current_end_pt != self.pt_d:
            if next == len(self.tuning_points)-1:
                control_info = ControlInfo()
                control_info.steering_angle = steering_angle
                control_info.desired_velocity = 0
                control_info.control_input = control_input
                self.control_info_pub.publish(control_info)
                print("finished running")            
            self.within_critical_range = True
            self.line_changed = False
            self.kp, self.ki, self.kd = self.curve_PIDs[self.next]
            lateral_error = (((self.position_x - self.circles[self.next][0])**2 + (self.position_z - self.circles[self.next][1])**2)**0.5 - self.circles[self.next][2])
            #print(self.ID, "in corner", lateral_error)

        #if a merging cav is near the merging point, switch to main path
        elif abs(self.position_x  - self.current_end_pt[0])  < self.ranges[self.next][0] and \
            abs(self.position_z - self.current_end_pt[1]) < self.ranges[self.next][1] and\
            self.current_end_pt == self.pt_d:

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
            self.current_end_pt = self.turning_pts[self.next]
            self.kp, self.ki, self.kd = self.PIDs[self.current]
            self.e_prev_lateral= 0
            self.e_int_lateral = 0
            lateral_error = (self.current_line[0]*self.position_x + self.current_line[1]*self.position_z + self.current_line[2])/((self.current_line[0]**2 + self.current_line[1]**2)**0.5)



        #increament collision points as they are traversed
        if abs(self.position_x  - self.current_collision_pt1[0])  < self.lane_width/2 and \
            abs(self.position_z - self.current_collision_pt1[1]) < self.lane_width/2:
            self.within_collision_range = True
            self.exit_collision_range = False
        else:
            self.exit_collision_range = True

        if self.within_collision_range and self.exit_collision_range:
            self.current_collision = (self.current_collision+1) % len(self.collision_pts)
            self.next_collision = (self.next_collision+1) % len(self.collision_pts)
            self.exit_collision_range = True
            self.within_collision_range = False
            self.current_collision_pt1 = self.collision_pts[self.current_collision]
            self.current_collision_pt2 = self.collision_pts[self.next_collision]
            
        
        
        
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
