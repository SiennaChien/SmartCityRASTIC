#!/usr/bin/env python3
# coding=UTF-8
import math
import numpy as np
from pylimo import limo
import rospy
import re
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive


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

    def control(self,e,v_ref,bias_x,eprev_lateral,eint_lateral,dt):
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
