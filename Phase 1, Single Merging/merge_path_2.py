import time
import rospy
from cav_for_merge_path import CAV

def main():
    #initialize CAV, PID values, and another parameters
    isMain = False #change isMain depending on if limo is running on main or merging path
    CAV1 = CAV("limo770")
    CAV1.generate_map(isMain)
    
    eprev_lateral_1= 0
    eint_lateral_1 = 0
    e = 0
    transmissionRate = 30
    dt = 1/transmissionRate # or 0.1
    v_ref_CAV1 = 0.5
    within_critical_range = False
    line_changed = True
    fast = 0.5
    slow = 0.3

    #depending on if the car starts at main path or merging path, initialize different starting paths, points, and PID values
    current = 0
    next = 1
    CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
    current_line = CAV1.lines[current]
    current_end_pt = CAV1.points[next]

    while True:

        #if the CAV is near a critical point (which are turning corners), set current line, starting point, and PID values to be that of the next line
        if abs(CAV1.position_x  - current_end_pt[0])  < CAV1.ranges[next][0] and \
           abs(CAV1.position_z - current_end_pt[1]) < CAV1.ranges[next][1] and\
            current_end_pt[0] != CAV1.merging_pt_x and current_end_pt[1] != CAV1.merging_pt_y:

            within_critical_range = True
            line_changed = False
            v_ref_CAV1 = slow
            CAV1.kp, CAV1.ki, CAV1.kd = CAV1.curve_PIDs[next]                
            e = (((CAV1.position_x - CAV1.circles[next][0])**2 + (CAV1.position_z - CAV1.circles[next][1])**2)**0.5 - CAV1.circles[next][2])
        
        #if a merging CAV is near the merging point, start to shift to main path
        elif abs(CAV1.position_x  - current_end_pt[0])  < CAV1.ranges[next][0] and \
           abs(CAV1.position_z - current_end_pt[1]) < CAV1.ranges[next][1] and\
           current_end_pt[0] == CAV1.merging_pt_x and current_end_pt[1] == CAV1.merging_pt_y:   
            within_critical_range = True
            line_changed = False
            current_line = CAV1.lines[next]
            e = -(current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            
        #when CAV is on a straight path
        else: 
            within_critical_range = False
            current_line = CAV1.lines[current]
            e = -(current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            v_ref_CAV1 = fast
        
        #once out of the turning point, increment i to be corrsponding to the new line
        if not line_changed and not within_critical_range: 
            current = (current+1) % len(CAV1.lines)
            next = (next+1) % len(CAV1.lines)
            line_changed = True
            within_critical_range = False
            v_ref_CAV1 = fast
            current_line = CAV1.lines[current]
            current_end_pt = CAV1.points[next]
            CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
            eprev_lateral_1= 0
            eint_lateral_1 = 0
            e = -(current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
        
        #calculate steering, pass onto listener on the limo
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1, eprev_lateral_1,eint_lateral_1,dt)
        CAV1.pub.publish(drive_msg_CAV1)

        time.sleep(dt)

if __name__ == '__main__':
    main()


