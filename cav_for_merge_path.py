#!/usr/bin/env python3
# coding=UTF-8
import math
# import numpy as np
# from pylimo import limo
import rospy
# import re
# import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive


class CAV():         

    def calc_distance(self, x1, y1, x2, y2):
        distance = ((x1-x2)**2 + (y1-y2)**2)**0.5
        return distance
    	
    def generate_map(self, isMain):
    	#in the RASTIC arena, these are the x and z values from the mocap of each critical point on the merging lane
        self.right_top_x = 3044
        self.right_top_y = 2536

        self.right_center_x = 2040
        self.right_center_y = 2519

        self.right_bottom_x = 550
        self.right_bottom_y = 2512

        self.left_top_x = 2933	
        self.left_top_y = -2036

        self.left_center_x = 1922
        self.left_center_y = -1977

        self.merging_pt_x = 1942
        self.merging_pt_y= -890

        self.lane_width = 450        

        #the ranges near each corner that activates the circle path for the limo to follow
        self.right_top_activation_range = (self.lane_width, self.lane_width)
        self.right_center_activation_range = (self.lane_width, self.lane_width)
        self.right_bottom_activation_range = (self.lane_width, self.lane_width*1.5)
        self.left_top_activation_range = (self.lane_width/2, self.lane_width/2)
        self.left_center_activation_range = (self.lane_width, self.lane_width)
        self.merging_pt_activation_range = (self.lane_width, self.lane_width)
            

        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.main_path = self.generate_line(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.return_first = self.generate_line(self.left_center_x, self.left_center_y, self.left_top_x, self.left_top_y)
        self.return_second = self.generate_line(self.left_top_x, self.left_top_y, self.right_top_x, self.right_top_y)
        self.return_third = self.generate_line(self.right_top_x, self.right_top_y, self.right_center_x, self.right_center_y)
        self.merging_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)
        self.off_path = self.generate_line(self.right_top_x, self.right_top_y, self.right_bottom_x, self.right_bottom_y) #return third to off path


        #data points that characterize each circle, center x, center y, radius, each variable is a tuple (A, B, C)
        self.right_top_circle = (self.right_top_x - self.lane_width, self.right_top_y - self.lane_width, self.lane_width)
        self.right_center_circle = (self.right_center_x + self.lane_width, self.right_center_y - self.lane_width, self.lane_width)
        self.right_bottom_circle = (self.right_bottom_x + self.lane_width, self.right_bottom_y - self.lane_width, self.lane_width/2)
        self.left_top_circle = (self.left_top_x - self.lane_width, self.left_top_y + self.lane_width, self.lane_width)
        self.left_center_circle = (self.left_center_x + self.lane_width, self.left_center_y + self.lane_width, self.lane_width)
        self.merging_circle = (self.merging_pt_x + self.lane_width, self.merging_pt_y - self.lane_width, self.lane_width) #in practice this is not use
        
        #distance of each path
        self.merging_path_dist = self.calc_distance(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y) 
        self.main_path_dist = self.calc_distance(self.merging_pt_x, self.merging_pt_y, self.left_center_x, self.left_center_y)
        self.half_main_path_dist = self.calc_distance(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.return_first_dist = self.calc_distance(self.left_center_x, self.left_center_y, self.left_top_x, self.left_top_y)
        self.return_second_dist = self.calc_distance(self.left_top_x, self.left_top_y, self.right_top_x, self.right_top_y)
        self.return_third_dist = self.calc_distance(self.right_top_x, self.right_top_y, self.right_center_x, self.right_center_y)
        self.off_path_dist = self.calc_distance(self.right_top_x, self.right_top_y, self.right_bottom_x, self.right_bottom_y)

        #PID values of each line, each element is a tuple (kp, ki, kd)
        self.merge_path_PID = (-0.0015, -0.000045, -0.0017) #-0.0015, -0.000045, -0.0017
        self.main_path_PID = (-0.0045, -0.000045, -0.0017) #-0.0045, -0.000045, -0.0017
        self.return_first_PID = (-0.0015, -0.000045, -0.0017) #-0.0015, -0.000045, -0.0017
        self.return_second_PID = (-0.0045, -0.000045, -0.0017) #-0.0045, -0.000045, -0.0017
        self.return_third_PID = (-0.0015, -0.000045, -0.0017) #-0.0015, -0.000045, -0.0017
        
        #PID values of each circle, each element is a tuple (kp, ki, kd)
        self.right_bottom_circle_PID = (-0.50, -0.00045, -0.037)
        self.right_center_circle_PID = (-0.0030, -0.000045, -0.0017)
        self.left_center_circle_PID= (-0.56, -0.00045, -0.037)
        self.left_top_circle_PID = (-0.60, -0.045, -0.050)
        self.right_top_circle_PID = (-0.55, -0.00045, -0.037)
        
        if isMain: #if the limo runs along the main path
            #array to store all points
            self.points = [(self.right_center_x, self.right_center_y), (self.left_center_x, self.left_center_y),
                        (self.left_top_x, self.left_top_y), (self.right_top_x, self.right_top_y)]
            #array to store all lines, in order of traversal
            self.lines = [self.main_path, self.return_first, self.return_second, self.return_third]
            #create an array that stores the distance of each line, in order of traversal
            self.dists = [self.main_path_dist, self.return_first_dist, self.return_second_dist, self.return_third_dist]
            #the activation range of the corners, in order of traversal
            self.ranges = [self.right_center_activation_range, self.left_center_activation_range, 
                           self.left_top_activation_range, self.right_top_activation_range]
            #array to store the circles for the corners, in order of traversal
            self.circles = [self.right_center_circle, self.left_center_circle, self.left_top_circle, self.right_top_circle]
            #array to store PID values of each line, in order of traversal, each element is a tuple (kp, ki, kd)
            self.PIDs = [self.main_path_PID, self.return_first_PID, self.return_second_PID, self.return_third_PID]
            #array to store PID values of each circle, in order of traversal, each element is a tuple (kp, ki, kd)
            self.curve_PIDs = [self.right_center_circle_PID, self.left_center_circle_PID, self.left_top_circle_PID, self.right_top_circle_PID]


        else: #if the limo runs along the merging path         
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y), (self.left_center_x, self.left_center_y),
                        (self.left_top_x, self.left_top_y), (self.right_top_x, self.right_top_y)]
            self.lines = [self.merging_path, self.main_path, self.return_first, self.return_second, self.off_path]
            self.dists = [self.merging_path_dist, self.half_main_path_dist, self.return_first_dist, self.return_second_dist, self.return_third_dist]
            self.PIDs = [self.merge_path_PID, self.main_path_PID, self.return_first_PID, self.return_second_PID, self.return_third_PID]
            self.ranges = [self.right_bottom_activation_range, self.merging_pt_activation_range, \
                           self.left_center_activation_range, self.left_top_activation_range, self.right_top_activation_range]
            self.circles = [self.right_bottom_circle, self.merging_circle, self.left_center_circle, self.left_top_circle, self. right_top_circle]
            self.curve_PIDs = [self.right_bottom_circle_PID, self.right_merging_circle_PID, self.left_center_circle_PID, self.left_top_circle_PID, self.right_top_circle_PID]
            
        
    def generate_line(self, x_1, y_1, x_2, y_2):
        A = -(y_2-y_1)
        B = -(x_1-x_2)
        C = -(y_1*(x_2-x_1)-(y_2-y_1)*x_1)
        return A, B, C

    
        
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
        self.kp=0
        self.ki=0
        self.kd = 0
        
        # construct publisher
        #rospy.init_node(self.node_name, anonymous=False)
        rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_'+self.node_name,AckermannDrive,queue_size=10) #topic name = CAV_Data
        rospy.Rate(10)

    def run (self):
        message = "%s %s %s %s" %(self.position_z, self.position_x, self.velocity, self.acceleration)
        self.pub.publish(message)

    def callback(self, msg):
        #Eul= self.quaternion_to_euler(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        self.position_z = msg.pose.position.z*1000
        #self.position_z = -msg.pose.position.y*1000
        self.position_x = msg.pose.position.x*1000
        self.position_yaw = 0 #Eul[2]
        self.Receivedata=1

    def steeringAngleToSteeringCommand(self,refAngle):
        x = refAngle
        y = 0.7*x
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

    def PIDController(self, x, x_ref, prev_e, prev_int, delta_t, Kp, Ki, Kd): #add theta_ref as input

        # Tracking error
        e = x_ref - x

        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int,0.5),-0.5)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # controller coefficients
        #Kp = .001
        #Ki = 0
        #Kd = 0.0001


        # PID controller for omega
        u_k = Kp*e
        u_i = Ki*e_int
        u_d = Kd*e_der
        u = Kp*e + Ki*e_int + Kd*e_der

        return u, u_k, u_i, u_d, e, e_int

    def PIDController(self, e, prev_e, prev_int, delta_t, Kp, Ki, Kd): #add theta_ref as input
        if e <= 1 and e>=-1:
            e_int = 0
        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int,0.3),-0.3)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # controller coefficients
        #Kp = .001
        #Ki = 0
        #Kd = 0.0001

        # PID controller for omega
        u_k = Kp*e
        u_i = Ki*e_int
        u_d = Kd*e_der
        u = Kp*e + Ki*e_int + Kd*e_der

        return u, u_k, u_i, u_d, e, e_int

    def control(self,e,v_ref, eprev_lateral,eint_lateral,dt):
        #e = self.position_x - bias_x #maybe its distance from the line we want to travel on?
        if (eprev_lateral*e<=0):
            eint_lateral = 0
        kp = self.kp
        ki = self.ki
        kd = self.kd

        [ref_steer,u_k ,u_i ,u_d, eprev_lateral, eint_lateral] = self.PIDController(e, eprev_lateral, eint_lateral, dt, kp, ki, kd)

        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = self.steeringAngleToSteeringCommand(ref_steer)
        return eprev_lateral,eint_lateral,drive_msg


    def control_dummy(CAV, v_ref,bias_x,bias_z,e,eprev_lateral,eint_lateral):
        z_ref = 424
        dt = 0.1 #maybe

        e = CAV.position_x - bias_x #maybe its distance from the line we want to travel on?
        [ref_steer,eprev_lateral,eint_lateral]=CAV.e2steer(e,eprev_lateral,eint_lateral,dt)

        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = CAV.steeringAngleToSteeringCommand(ref_steer)
        return e,eprev_lateral,eint_lateral,drive_msg
