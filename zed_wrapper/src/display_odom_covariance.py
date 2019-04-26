#!/usr/bin/env python
import rospy
from tf.transformations import *
import numpy as np
import math


from nav_msgs.msg import Odometry

def callback(zed1):
# do my stuff

	x1 = zed1.pose.pose.position.x
	y1 = zed1.pose.pose.position.y
	z1 = zed1.pose.pose.position.z
	q1x = zed1.pose.pose.orientation.x
	q1y = zed1.pose.pose.orientation.y
	q1z = zed1.pose.pose.orientation.z
	q1w = zed1.pose.pose.orientation.w
	(r1, p1, ya1) = euler_from_quaternion((q1x, q1y, q1z, q1w))
	cov1 = zed1.pose.covariance

	print "\nposition(m): %6.3f %6.3f %6.3f   orientation(degs): %6.1f %6.1f %6.1f " % (x1, y1 ,z1, r1*r2d, p1*r2d, ya1*r2d)	
#	print "cov1: %9.7f" % np.max(cov1)
	print "covariance: %8.5f %8.5f %8.4f %8.5f %8.5f %8.5f" % (cov1[0], cov1[7], cov1[14], cov1[21],cov1[28], cov1[35])

def listener():
	rospy.init_node('display_covariance',anonymous=True)
	rospy.Subscriber('/zed/odom',Odometry,callback)
	rospy.spin()

if __name__ == '__main__':
	r2d = 180./3.141592
	cov = [0,0,0,0,0,0]
	pose = [0,0,0,0,0,0]
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
