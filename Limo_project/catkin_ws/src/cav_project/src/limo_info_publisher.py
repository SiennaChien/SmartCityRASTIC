#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cav_project.msg import limo_info, limo_info_array


class LimoInfoPublisher:
    def __init__(self):
        rospy.init_node('limo_info_publisher')
        self.info_pub_cav1 = rospy.Publisher('/limo_info_cav1', limo_info, queue_size=10)
        self.info_pub_cav2 = rospy.Publisher('/limo_info_cav2', limo_info, queue_size=10)
        self.odom_sub_cav1 = rospy.Subscriber('/odom_cav1', Odometry, self.odom_callback_cav1)
        self.odom_sub_cav2 = rospy.Subscriber('/odom_cav2', Odometry, self.odom_callback_cav2)
        self.mocap_sub_cav1 = rospy.Subscriber('/vrpn_client_node/limo770/pose', PoseStamped, self.mocap_callback_cav1)
        self.mocap_sub_cav2 = rospy.Subscriber('/vrpn_client_node/limo155/pose', PoseStamped, self.mocap_callback_cav2)
        self.task_sub = rospy.Subscriber('/task', String, self.task_callback)

        self.limo_data_cav1 = limo_info()
        self.limo_data_cav1.ID.data = 770
        self.limo_data_cav2 = limo_info()
        self.limo_data_cav2.ID.data = 155
        self.limo_array_data = limo_info_array()
        self.path_string = 'NONE'
        self.rate = rospy.Rate(10)

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
        self.limo_array_data.limo_infos = [self.limo_data_cav1, self.limo_data_cav2]
        # Uncomment if you need to publish the array data somewhere
        # self.limo_array_pub.publish(self.limo_array_data)

    def odom_callback_cav1(self, msg):
        self.limo_data_cav1.vel.data = msg.twist.twist.linear.x
        self.publish_info()

    def odom_callback_cav2(self, msg):
        self.limo_data_cav2.vel.data = msg.twist.twist.linear.x
        self.publish_info()

    def mocap_callback_cav1(self, msg):
        if self.path_string == "NONE":
            return
        self.limo_data_cav1.x.data = msg.pose.position.x
        self.limo_data_cav1.y.data = msg.pose.position.y
        self.limo_data_cav1.origin_dist.data = msg.pose.position.z
        self.publish_info()

    def mocap_callback_cav2(self, msg):
        if self.path_string == "NONE":
            return
        self.limo_data_cav2.x.data = msg.pose.position.x
        self.limo_data_cav2.y.data = msg.pose.position.y
        self.limo_data_cav2.origin_dist.data = msg.pose.position.z
        self.publish_info()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    publisher = LimoInfoPublisher()
    publisher.run()

