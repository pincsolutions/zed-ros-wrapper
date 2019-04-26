#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf.transformations import *

class SwapXY(object):
    def __init__(self):
        self.zed_odom_publisher = rospy.Publisher("/zed/swapped_odom", Odometry, queue_size=10)
    def zed_odom_callback(self, data):
    	"""
    	Odometry callback that swaps the x and y pose
    	"""
        data.pose.pose.position.x, data.pose.pose.position.y = -data.pose.pose.position.y, data.pose.pose.position.x
	data.pose.pose.orientation.x, data.pose.pose.orientation.y = -data.pose.pose.orientation.y, data.pose.pose.orientation.x
	
 	x1 = data.pose.pose.position.x
	y1 = data.pose.pose.position.y
	z1 = data.pose.pose.position.z
	q1x = data.pose.pose.orientation.x
	q1y = data.pose.pose.orientation.y
	q1z = data.pose.pose.orientation.z
	q1w = data.pose.pose.orientation.w
	(r1, p1, ya1) = euler_from_quaternion((q1x, q1y, q1z, q1w))
	cov1 = data.pose.covariance  #Need to swaped for xy

	print "\nPose   x: %4.2f  y: %4.2f  z: %4.2f   Rx: %4.1f  Ry: %4.1f  Rz: %4.1f " % (x1, y1 ,z1, r1*r2d, p1*r2d, ya1*r2d)
	print "covariance: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f" % (cov1[7], cov1[0], cov1[14], cov1[21],cov1[28], cov1[35])

        self.zed_odom_publisher.publish(data)
	

if __name__ == '__main__':
    r2d = 180./3.141592
    rospy.init_node("swap_odometry_xy_node")
    # rospy.sleep(10)
    swap_xy = SwapXY()
    rospy.Subscriber('/zed/odom', Odometry, swap_xy.zed_odom_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
