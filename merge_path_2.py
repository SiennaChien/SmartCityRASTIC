import numpy as np
import matplotlib.pyplot as plt
#import pygame
#import socket
import time
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
import rospy
from cav_for_merge_path import CAV

def main():
    #initialize CAV, PID values, and another parameters
    isMain1 = False
    CAV1 = CAV("limo770")
    CAV1.generate_map(isMain1)
    
    # isMain2 = True
    # CAV2 = CAV("limo155")
    # CAV2.generate_map(isMain2)
    
    eprev_lateral_1= 0
    eint_lateral_1 = 0
    e = 0
    transmissionRate = 30
    dt = 1/transmissionRate # or 0.1
    #rate = rospy.Rate(transmissionRate) # 1Hz
    v_ref_CAV1 = 0.5 # set between 0.5 and 0.6
    within_critical_range = False
    line_changed = True

    #depending on if the car starts at main path or merging path, initialize different starting paths, points, and PID values
    current = 0
    next = 1
    CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
    current_line = CAV1.lines[current]
    current_end_pt = CAV1.points[next]

    # access the array that stores the distance of each line, then change velocity if the length is quite large
    # v_ref_CAV1 = 0.5 # set between 0.5 and 0.6, or higher if line is longer

    while True:

        #if the cav is near a critical point (which are turning corners), set current line, starting point, and PID values to be that of the next line
        if abs(CAV1.position_x  - current_end_pt[0])  < CAV1.ranges[next][0] and \
           abs(CAV1.position_z - current_end_pt[1]) < CAV1.ranges[next][1] and\
            current_end_pt[0] != CAV1.merging_pt_x and current_end_pt[1] != CAV1.merging_pt_y:

            within_critical_range = True
            line_changed = False
            v_ref_CAV1 = 0.3
            CAV1.kp, CAV1.ki, CAV1.kd = CAV1.curve_PIDs[next]                
            e = (((CAV1.position_x - CAV1.circles[next][0])**2 + (CAV1.position_z - CAV1.circles[next][1])**2)**0.5 - CAV1.circles[next][2])
            print("in corner", e)
        
        elif abs(CAV1.position_x  - current_end_pt[0])  < CAV1.ranges[next][0] and \
           abs(CAV1.position_z - current_end_pt[1]) < CAV1.ranges[next][1] and\
           current_end_pt[0] == CAV1.merging_pt_x and current_end_pt[1] == CAV1.merging_pt_y:   
            within_critical_range = True
            line_changed = False
            current_line = CAV1.lines[next]
            e = -(current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            print("in corner", e)
             
        else: 
            within_critical_range = False
            current_line = CAV1.lines[current]
            e = -(current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            v_ref_CAV1 = 0.6
            # if CAV1.dists[current] < 3000:
            #  	v_ref_CAV1 = 0.5
            # else:
            #     v_ref_CAV1 = 0.7
             	
            print("out of corner", e)     
        
        #once out of the turning point, increment i to be corrsponding to the new line
        if not line_changed and not within_critical_range: 
            print("changing lines", e)
            current = (current+1) % len(CAV1.lines)
            next = (next+1) % len(CAV1.lines)
            line_changed = True
            within_critical_range = False
            v_ref_CAV1 = 0.5
            current_line = CAV1.lines[current]
            current_end_pt = CAV1.points[next]
            CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
            eprev_lateral_1= 0
            eint_lateral_1 = 0
            e = -(current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
        
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1, eprev_lateral_1,eint_lateral_1,dt)
        CAV1.pub.publish(drive_msg_CAV1)

        time.sleep(dt)

    rospy.spin()

if __name__ == '__main__':
    main()


