#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):


        self.pub = rospy.Subscriber('laser_data', LaserScan, self.callback_laser)
        self.cmd = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)

        # TODO:
        # Initialize your publishers and
        # subscribers here
        

        rospy.spin()

    # TODO:
    # Write your callback functions here.
    def callback_laser(LaserScan):
        data = LaserScan.ranges
        rospy.loginfo((LaserScan.angle_min, LaserScan.angle_max, LaserScan.angle_increment))
        # self.pub = rospy.Publisher(data, LaserData)
        rospy.loginfo(datas)

        # We want to divide up the laserscan data
        #pub = rospy.Publisher('laser_data/left')
        #pub = rospy.Publisher('laser_data/front')
        #pub = rospy.Publisher('laser_data/right')
    def steering():
        # rate = rospy.Rate(10)

        # while not rospy.is_shutdown():
        steer_cmd = AckermannDriveStamped()
        steer_cmd.header.stamp = rospy.Time.now()
        steer_cmd.frame_id = 'base_link'
        steer_cmd.steering_angle = 0
        steer_cmd.speed = VELOCITY
        rospy.loginfo(steer_cmd)
        self.cmd.publish(steer_cmd)
        # rate.sleep

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()