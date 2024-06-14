import numpy as np
import time
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
import rospy
from pid import PID as CAV


def main():

    CAV1 = CAV("limo795")
    CAV1.kp = -0.0015 #-.001 # -.004
    CAV1.ki = -0.000045 #-0.000035 #-.0004
    CAV1.kd = -0.0015 #-0.001 #-.0055
    transmissionRate = 30
    rate = rospy.Rate(transmissionRate) # 1Hz

    bias_x1  = CAV1.position_x
    bias_z1 = CAV1.position_z
    #print(bias_x2,bias_x3)
    eprev_lateral_1= 0
    eint_lateral_1 = 0
    dt = 0.1
    mp_x = 2860
    mp_z = 6151

    while True:

        #v_ref_CAV2 = .45
        v_ref_CAV1 = 0.5

        #print(drive_msg_HDV1)
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(v_ref_CAV1,bias_x1,eprev_lateral_1,eint_lateral_1,dt)

        CAV1.pub.publish(drive_msg_CAV1)
        time.sleep(dt)
        print(CAV1.position_x)

    rospy.spin()

if __name__ == '__main__':
    main()
