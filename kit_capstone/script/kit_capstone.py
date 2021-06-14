#!/usr/bin/env python

import rospy
import math

#from kit_capstone.msg import Drive
from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import Point, Twist, Pose
from visualization_msgs.msg import MarkerArray, Marker


class CapstoneMainDrive:
	def __init__(self):
                self.dotest = rospy.get_param("/dotest", True)

                self.marker_idx = 0
                self.markerArray = MarkerArray()
		self.count = 0

                # init
		rospy.init_node('kit_capstone', anonymous=True)
		self.lidar_sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_callback) 
		self.steer_pub = rospy.Publisher('controller', Twist, queue_size = 10)
		self.marker_pub = rospy.Publisher('marker', MarkerArray, queue_size = 10)

	def obstacles_callback(self,data):
		rospy.loginfo('callback_start')
		self.main_pathing(data)
		
	def waypoint(self, data):
                way_point = Point()

                for obs in data.segments:
                    way_point.x += obs.first_point.x
                    way_point.x += obs.last_point.x

                    way_point.y += obs.first_point.y
                    way_point.y += obs.last_point.y

                way_point.x = way_point.x / (len(data.segments) * 2)
                way_point.y = way_point.y / (len(data.segments) * 2)
                
                self.init_markers(way_point.x, way_point.y)

                #return way_point.x, way_point.y
                return math.atan(way_point.y/way_point.x)


	def calc_angle(self, data):
		return self.waypoint(data) * 180 / math.pi


        def calc_velocity(self, angle):

                if (abs(angle) > 8):
		    return 1
		else:
		    return 2


        def main_pathing(self, data):
                drive_data = Twist()

                if self.dotest:
		    drive_data.angular.z = self.calc_angle(data)
                    drive_data.linear.x = self.calc_velocity(drive_data.angular.z)

                    rospy.loginfo("angle = " + str(drive_data.angular.z))
                    rospy.loginfo("velocity= " + str(drive_data.linear.x))

                    self.steer_pub.publish(drive_data)


        def init_markers(self,x,y):
                marker = Marker()
                marker.header.frame_id = 'laser'
		marker.action = marker.ADD
                marker.type = marker.SPHERE
		marker.pose.orientation.w = 1.0
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0

                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

		if (self.count > 0):
			self.markerArray.markers.pop(0)

		self.markerArray.markers.append(marker)

		id = 0

		for m in self.markerArray.markers:
			m.id = id
			id += 1

		self.count += 1

                self.marker_pub.publish(self.markerArray)


if __name__ == '__main__':
	try:
		capstone_drive = CapstoneMainDrive()
		rospy.spin()

	except rospy.ROSInterruptException:
		print(error)
		pass
