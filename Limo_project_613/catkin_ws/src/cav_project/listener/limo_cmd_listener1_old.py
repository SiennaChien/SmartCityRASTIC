#!/usr/bin/env python3

from pylimo import limo
import rospy
from ackermann_msgs.msg import AckermannDrive
import time
import numpy as np
import pickle
from cav_project.msg import limo_info, QP_solution, ControlInfo

class LimoInfoPublisher:
    def __init__(self):
        rospy.init_node('limo_info_publisher')
        self.info_pub_cav1 = rospy.Publisher('/limo_info_cav1', limo_info, queue_size=10)
        self.odom_sub_cav1 = rospy.Subscriber('/odom_cav1', Odometry, self.odom_callback_cav1)
        self.mocap_sub_cav1 = rospy.Subscriber('/vrpn_client_node/limo770/pose', PoseStamped, self.mocap_callback_cav1)
        self.control_info_sub = rospy.Subscriber("control_info_cav1", ControlInfo, self.callback)
        self.task_sub = rospy.Subscriber('/task', String, self.task_callback)

        self.limo_data_cav1 = limo_info()
        self.limo_data_cav1.ID.data = 770
        self.limo_array_data = limo_info_array()
        self.path_string = 'NONE'
        self.rate = rospy.Rate(10)
    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.data = data

    def task_callback(self, msg):
        path = msg.data
        lane_s = path[1]
        turn_s = path[0]
        self.path_string = 'straight_' + lane_s
        self.limo_data_cav1.path = String(self.path_string)
        self.limo_data_cav2.path = String(self.path_string)
        self.publish_info()

    def publish_info(self):
        self.info_pub_cav1.publish(self.limo_data_cav1)
        self.info_pub_cav2.publish(self.limo_data_cav2)
        self.limo_array_data.limo_infos = [self.limo_data_cav1]
        # Uncomment if you need to publish the array data somewhere
        # self.limo_array_pub.publish(self.limo_array_data)

    def odom_callback_cav1(self, msg):
        self.limo_data_cav1.vel.data = msg.twist.twist.linear.x
        self.publish_info()

    def mocap_callback_cav1(self, msg):
        if self.path_string == "NONE":
            return
        self.limo_data_cav1.x.data = msg.pose.position.x
        self.limo_data_cav1.y.data = msg.pose.position.y
        self.limo_data_cav1.origin_dist.data = msg.pose.position.z
        self.publish_info()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("limo_node", anonymous=True)
    listener_ins = listener()
    #publisher_ins = publisher()
    limo= limo.LIMO()
    limo.EnableCommand()
    limo.SetMotionCommand(linear_vel=0, steering_angle=0)
    time.sleep(1)
    steering_angle = np.zeros(250)
    imu_yaw =  np.zeros(250)
    iter = 0
    while True:
        listener_ins.listener()
        if listener_ins.data is not None:
            #listener_ins.data.speed
            #listener to message and set velocity and steering
            limo.SetMotionCommand(linear_vel=listener_ins.data.desired_velocity, steering_angle=listener_ins.data.steering_angle)
            print("Limo velocity command:", listener_ins.data.desired_velocity, "Limo steering:", listener_ins.data.steering_angle )
            #steering_angle[iter] = limo.GetSteeringAngle()
           # imu_yaw[iter] = limo.GetIMUYawData()
            iter += 1
            time.sleep(0.1)
        else:
            print("[WARNING] Skipping ros messages...")
            if iter>10:
                break
    outputFile = 'steering.pickle'
    data = {'imu_yaw':imu_yaw, 'steer_limo':steering_angle}
    fw = open(outputFile, 'wb')
    pickle.dump(data, fw)
    fw.close()
