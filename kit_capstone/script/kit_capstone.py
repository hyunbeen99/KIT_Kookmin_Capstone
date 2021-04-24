#!/usr/bin/env python

import rospy

from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

class CapstoneMainDrive:
	def __init__(self):
		rospy.init_node('kit_capstone', anonymous=True)

		self.lidar_sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_callback) 
		#self.imu_sub = rospy.Subscriber('kit_capstone_imu', , ) 
		#self.steer_pub 

	def obstacles_callback(self,data):
		rospy.loginfo('callback_start')
		self.main_pathing(data)
		

	def main_pathing(self, data):
		rospy.loginfo('Drive_start')



if __name__ == '__main__':
	try:
		capstone_drive = CapstoneMainDrive()
		rospy.spin()

	except rospy.ROSInterruptException:
		print(error)
		pass
