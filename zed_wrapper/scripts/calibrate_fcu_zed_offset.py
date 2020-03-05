#!/usr/bin/env python
import rospy
import time
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
 
class Calibrate_IMU_Offset:

	def __init__(self):

		self.fcu_sub = rospy.Subscriber('/mavros/imu/data',Imu , self.fcu_imu_cb)
		self.zed1_imu_sub = rospy.Subscriber('/zed1/imu/data',Imu,self.zed1_imu_cb)
		self.zed2_imu_sub = rospy.Subscriber('/zed2/imu/data',Imu, self.zed2_imu_cb)

		self.fcu_imu = np.array([0., 0., 0.])
		self.zed1_imu = np.array([0., 0., 0.])
		self.zed2_imu = np.array([0., 0., 0.])

	def fcu_imu_cb(self, msg):
 		euler = euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
		self.fcu_imu =  np.array(euler)*57.2958 
		#print "fcu:  ", self.fcu_imu
		return 

	def zed1_imu_cb(self, msg):
                euler = euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
		self.zed1_imu =  np.array(euler)*57.2958
  		#print "zed1: ", self.zed1_imu
		return

	def zed2_imu_cb(self, msg):
                euler = euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
                self.zed2_imu =  np.array(euler)*57.2958	
		#print "zed2: ", self.zed2_imu
		return

	def take_average(self):
		n = 0
		sum0 = np.array([0., 0., 0.])
		sum1 = sum0
		sum2 = sum0 
		time.sleep(1)
		while n<40:
			print "fcu:  ", self.fcu_imu
			sum0 = sum0 + self.fcu_imu
			print "zed1: ", self.zed1_imu
			sum1 = sum1 + self.zed1_imu
			print "zed2: ", self.zed2_imu
			sum2 = sum2 + self.zed2_imu
			time.sleep(0.5)
                        print "n = ", n
			n = n + 1
                avg_fcu_imu = sum0/40.
		avg_zed1_imu = sum1/40.
		avg_zed2_imu = sum2/40.

		urdf2 = avg_zed2_imu - avg_fcu_imu
		urdf1 = -avg_zed1_imu - avg_fcu_imu 
		
		print "\naverage fcu_imu: ", avg_fcu_imu
		print "average zed1_imu: ", avg_zed1_imu
		print "average zed2_imu: ", avg_zed2_imu
		
		print "\ncalculated urdf2 in deg: ", urdf2
		print "calculated urdf2 in rad: ", urdf2/57.2958
		print "calculated urdf1 in deg: ", urdf1
		print "calculated urdf1 in rad: ", urdf1/57.2958


if __name__ == '__main__':
	rospy.init_node('calibrate_imu_offset', anonymous=False)
	calibrate = Calibrate_IMU_Offset()
	calibrate.take_average()
	rospy.spin()
