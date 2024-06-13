import numpy as np
import matplotlib.pyplot as plt
import pygame
import logitechG29_wheel
import socket
import time
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
import rospy
from cav_test import CAV
from steering_demo_comms_dept import manual as HDV



def vel_ip_rear(z_i,z_ip):

    if z_ip-z_i>= 1500:
    	v=0.8
    else:
        v =  0.0008888889*(z_ip-z_i) - 0.5333333
        #print(v)
    return v


def vel_i_merg(z_i,x_i,z_i1,x_i1,mp_z,mp_x,bias_z):
    p_i_abs=z_i-bias_z #position along the main lane
    p_i_m=mp_z-z_i
    p_i1_m=np.sqrt((x_i1-mp_x)**2+(z_i1-mp_z)**2) #position from the merging point
    L = 6151 - 767
    print(z_i,x_i,z_i1,x_i1)
    if x_i1 < 2600:
        if p_i_m-p_i1_m>= 1500:
            print("1")
            v = 0.8
        else:
            print("2")
            v = min(max(0.8/(1500-600)*(p_i_m-p_i1_m-600*p_i_abs/L),0),0.8)
    else:
        if z_i1-z_i>= 1500:
            print("3")
            v = 0.8
        else:
            print("4")
            v =  0.0008888889*(z_i1-z_i) - 0.5333333

    return v
    
    
    
def line_coef(x_1, x_2, y_1, y_2):
    A = y_1-y_2
    B = x_2-x_1
    C = x_1*y_2-x_2*y_1
    return A, B, C


def main():

    CAV1 = CAV("limo770")
    CAV1.kp = -0.0015 #-.001 # -.004
    CAV1.ki = -0.000045 #-0.000035 #-.0004
    CAV1.kd = -0.0017 #-0.001 #-.0055
    transmissionRate = 30
    rate = rospy.Rate(transmissionRate) # 1Hz
    
    bias_x1  = CAV1.position_x
    bias_z1 = CAV1.position_z
    
    #print(bias_x2,bias_x3)
    eprev_lateral_1= 0
    eint_lateral_1 = 0
    eprev_lateral_2= 0
    eint_lateral_2 = 0
    eprev_lateral_3= 0
    eint_lateral_3 = 0
    dt = 0.1
    mp_x = 2860
    mp_z = 6151
    
    
    z_ref = 424
    bias = 0
    e = 0
    e_int = 0
    eprev_lateral= 0
    eint_lateral = 0
    transmissionRate = 30
    dt = 1/transmissionRate
    cnt = 0
    i = 0
    v_ref = 0.5
    v = 0
    count = 0
    
    
    v_ref_CAV1 = 0.5
    p1_x = 2040
    p1_y = 2519

    p2_x = 1922
    p2_y = -1937
    
    [A,B,C] = line_coef(p1_x,p2_x,p1_y,p2_y)

    while True:
        
        #print(drive_msg_HDV1)
        e = -(A*CAV1.position_x + B*CAV1.position_z + C)/((A**2 + B**2)**0.5)
        
        #print(CAV1.position_x)
        #print(CAV1.position_z)
        print(e)
        
        
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1,bias_x1,eprev_lateral_1,eint_lateral_1,dt)
	

        CAV1.pub.publish(drive_msg_CAV1)
        
        time.sleep(dt)

    rospy.spin()

if __name__ == '__main__':
    main()
