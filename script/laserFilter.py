#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoFilter:
    def __init__(self):

        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("scan_filtered", LaserScan, queue_size=10)

    def callback(self, data):
        newdata = data
        newdata.ranges = list(data.ranges)
        #newdata.intensities = int(list(255-data.ranges*20))
	newdata.intensities = list(data.intensities)
        for x in range(150,210):
            newdata.ranges[x]=float('inf')
            newdata.intensities[x]=0

        self.pub.publish(newdata)

if __name__ == '__main__':
    rospy.init_node('laserFilter', anonymous=False)
    lidar = DoFilter()

    rospy.spin()

