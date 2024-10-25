import time
from ackermann_msgs.msg import AckermannDrive
from cav_for_PID_tuning import CAV

def main():
    #initialize CAV, PID values, and another parameters
    CAV1 = CAV("limo795")
    CAV1.generate_map('e')

    eprev_lateral_1= 0
    eint_lateral_1 = 0
    e = 0
    transmissionRate = 30
    dt = 1/transmissionRate # or 0.1
    v_ref_CAV1 = 0.5 # set between 0.5 and 0.6
    within_critical_range = False
    line_changed = True

    #depending on if the car starts at main path or merging path, initialize different starting paths, points, and PID values
    current = 0
    next = 1
    CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
    current_line = CAV1.lines[current]
    current_end_pt = CAV1.turning_pts[next]
        
    while True:
        #if the cav is near a critical point (which are turning corners), set path to a circle, change starting point and PID values to fit
        if abs(CAV1.position_x  - current_end_pt[0])  < CAV1.ranges[next][0] and \
           abs(CAV1.position_z - current_end_pt[1]) < CAV1.ranges[next][1]:
            if (next == len(CAV1.turning_pts) -1): 
                CAV1.regenerate_map()
                current = 0
                next = 1
                CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
                current_line = CAV1.lines[current]
                current_end_pt = CAV1.turning_pts[next]
            within_critical_range = True
            line_changed = False
            current_line = CAV1.lines[current]
            e = (((CAV1.position_x - CAV1.circles[next][0])**2 + (CAV1.position_z - CAV1.circles[next][1])**2)**0.5 - CAV1.circles[next][2])
            print("turning")

        #when the cav is on a straight path
        else:
            within_critical_range = False
            current_line = CAV1.lines[current]
            e = (current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            print("straight path, error:", e)

        #once out of the turning point, follow the next line
        if not line_changed and not within_critical_range:
            current = current+1
            next = next+1
            line_changed = True
            within_critical_range = False
            current_line = CAV1.lines[current]
            current_end_pt = CAV1.turning_pts[next]
            CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
            eprev_lateral_1= 0
            eint_lateral_1 = 0
            e = (current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)

            
                
        #calculate steering and publisher to the listener node on the limo
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1, eprev_lateral_1,eint_lateral_1,dt)
        CAV1.pub.publish(drive_msg_CAV1)

        time.sleep(dt)
        
def calc_distance(pt_1, pt_2):
        distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
        return distance

if __name__ == '__main__':
    main()
