import time
from cav_for_line_follower import CAV

#helper function to get equation of a line in the form Ax + By + C = 0
def line_coef(x_1, x_2, y_1, y_2):
    A = y_1-y_2
    B = x_2-x_1
    C = x_1*y_2-x_2*y_1
    return A, B, C

def main():

    #creating cav object, setting PID values
    CAV1 = CAV("limo770")
    CAV1.kp = -0.0015 
    CAV1.ki = -0.000045
    CAV1.kd = -0.0017  
    eprev_lateral_1= 0
    eint_lateral_1 = 0
    dt = 0.1
    e = 0
    transmissionRate = 30
    dt = 1/transmissionRate
    v_ref_CAV1 = 0.5

    #the two points that define the line to travel on
    p1_x = 3044
    p1_y = 2536

    p2_x = 683
    p2_y = 2536
    
    #calculates equation of line
    [A,B,C] = line_coef(p1_x,p2_x,p1_y,p2_y)

    while True:
        #calculate error, get steering, publish to limo
        e = -(A*CAV1.position_x + B*CAV1.position_z + C)/((A**2 + B**2)**0.5)
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1,CAV1.position_x,eprev_lateral_1,eint_lateral_1,dt)
        CAV1.pub.publish(drive_msg_CAV1)

        time.sleep(dt)

if __name__ == '__main__':
    main()
