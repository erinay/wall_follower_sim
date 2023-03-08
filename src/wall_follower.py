#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rospy_tutorials.msg import Floats
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    kp = 5.8
    kd = 4.55
    def __init__(self):

        # Initialize your publishers and
        # subscribers here
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback_laser)
        self.cmd = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.pub_scan = rospy.Publisher('/scan_data', numpy_msg(Floats), queue_size=10)
        self.pub_scanf = rospy.Publisher('/scan_data', numpy_msg(Floats), queue_size=10)
        self.line_pub = rospy.Publisher('wall', Marker, queue_size=10)
        
        rate = rospy.Rate(10)

       # Write your callback functions here.
    def callback_laser(self, LaserScan):
    # Get laser data and publish 
        data = np.array(LaserScan.ranges)
        angle_min = LaserScan.angle_min
        angle_max = LaserScan.angle_max
        angle_increment = LaserScan.angle_increment
        angles = np.arange(angle_min, angle_max, angle_increment)

        index = np.where(abs(angles)<np.pi/2)
        angles = angles[index]
        data = data[index]
        num_samples = int(np.size(index)/2)

        # rospy.loginfo(angle_increment)
        # Slice range data to three sections and publish
        # # Position wrt to base_link frame
        # # # rospy.loginfo(data.size)
        # used_anglef = angles[35:65]
        # scan_ftdata = data[35:65]


        if(self.SIDE==-1):
            scan_walldata = data[index[0][0]-5:num_samples+10]
            used_angle = angles[index[0][0]-5:num_samples+10]

        else:
            scan_walldata = data[num_samples-10:index[0][-1]+5]
            used_angle = angles[num_samples-10:index[0][-1]+5]


        tempwall = np.multiply(scan_walldata, np.sin(used_angle))
        
        ind = np.where(scan_walldata>(self.DESIRED_DISTANCE*2.5))
        
        # ind2 = np.where(tempwall>(self.DESIRED_DISTANCE*2.5))
        # rospy.loginfo(ind)


        
        # for indices in range(np.size(scan_walldata)):
        #     if ((indices in ind) and (indices in ind2)):
        #         rmp.append(indices)
        np.delete(ind, scan_walldata)
        # np.delete(np.array(indices), used_angle)
        # rospy.loginfo(np.size(wall))
        # rospy.loginfo(wall)

        # tempft = np.multiply(scan_ftdata, np.cos(used_anglef))
        # # rospy.loginfo(tempft)cos
        # indexf = np.where(tempft>(self.DESIRED_DISTANCE*2))
        # rospy.loginfo(np.size(indexf))
        # scan_ftdata = np.delete(scan_ftdata, indexf)
        # used_anglef = np.delete(used_anglef, indexf)

        if (np.size(scan_walldata)<2):
            rospy.loginfo('straight')
            self.steering(0,0,1)
            return

        # elif np.size(scan_walldata)<2 and np.size(scan_ftdata)>2:
        #     rospy.loginfo('wall in front')
        #     self.steering(0,0,2)
        #     return
        
        else: #np.size(scan_walldata)>4 and np.size(scan_ftdata)<8:
        # else:
            # rospy.loginfo('naviagte')

            # fwallx = np.multiply(scan_ftdata, np.cos(used_anglef))
            # fwally = np.multiply(scan_ftdata, np.sin(used_anglef))
            
            xwall = scan_walldata*np.cos(used_angle)
            # if(self.SIDE==1):
            ywall = scan_walldata*np.sin(used_angle)
            # else:
            #     ywall = np.multiply(scan_walldata, np.sin(used_angle))

            # xwall = np.hstack((wallx, fwallx))
            # ywall = np.hstack((wally, fwally))
            
        # else:
        #     wallx = np.multiply(scan_walldata, np.cos(used_angle))
        #     wally= np.multiply(scan_walldata, np.sin(used_angle))
        #     fwallx = np.multiply(scan_ftdata, np.cos(used_anglef))
        #     fwally = np.multiply(scan_ftdata, np.sin(used_anglef))
        #     rospy.loginfo('steer around corner')
        #     wall = np.hstack((scan_walldata, scan_ftdata))
        #     ang = np.hstack((used_angle, used_anglef))
        #     xwall = np.multiply(wall, np.cos(ang))
        #     ywall = np.multiply(wall, np.sin(ang))
        #     xwall = np.hstack((wallx, fwallx))
        #     ywall = np.hstack((wally, fwally))
        #     rospy.loginfo(np.size(scan_walldata))

            # Extract points (wall)
            

        [slope, intercept] = np.polyfit(xwall, ywall,1)

        # Draw Line
        xmin = min(xwall)
        xmax = max(xwall)
        x=np.arange(xmin,xmax, 0.1)
        y= slope*x + intercept
        # y = [ymin, ymax]
        
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")
        mean = np.mean(xwall)
        self.steering(slope,intercept)
  

    def steering(self, slope, coeff, override=0):
    
        # kp porportional error term
        error = (self.DESIRED_DISTANCE-abs(coeff))
        rospy.loginfo(error)

        try:
            des_theta = np.tan(slope)  
        except:
            des_theta = -pi/2

        controller = (self.kp*error*self.SIDE-self.kd*slope*self.VELOCITY)*-1.
        # -self.kd*slope
        steer_cmd = AckermannDriveStamped()
        steer_cmd.header.stamp = rospy.Time.now()
        steer_cmd.header.frame_id = 'base_link'
        steer_cmd.drive.steering_angle = controller
        steer_cmd.drive.speed = self.VELOCITY
        # if (override==1):
        #     steer_cmd.drive.steering_angle = 0
        # elif(override==2):
        #     error = np.pi/2
        #     steer_cmd.drive.steering_angle = self.kp*error+self.kd*slope
        rospy.loginfo(steer_cmd)
        self.cmd.publish(steer_cmd)

    def draw_wall(self, x, y):
        # Apply lin lig
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
