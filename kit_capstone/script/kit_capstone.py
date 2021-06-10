#!/usr/bin/env python

import rospy
import math

#from kit_capstone.msg import Drive
from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import Point, Twist, Pose
from visualization_msgs.msg import MarkerArray, Marker

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def totalError(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error


class CapstoneMainDrive:
	def __init__(self):
                # Parameter
                kp = rospy.get_param("/kp", 0.2)
                ki = rospy.get_param("/ki", 0.005)
                kd = rospy.get_param("/kd", 0.05)

                self.dotest = rospy.get_param("/dotest", True)
                self.padding_angle = rospy.get_param("/padding_angle", 10)
                self.padding_velocity = rospy.get_param("/padding_velocity", 2)

                self.pid = PID(kp, ki, kd)

                self.marker_idx = 0
                self.markerArray = MarkerArray()
		self.count = 0

                # init
		rospy.init_node('kit_capstone', anonymous=True)
		self.lidar_sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_callback) 
		self.steer_pub = rospy.Publisher('controller', Twist, queue_size = 10)
		self.marker_pub = rospy.Publisher('marker', MarkerArray, queue_size = 10)
		#self.imu_sub = rospy.Subscriber('kit_capstone_imu', , ) 

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

                way_point.x /= len(data.segments)
                way_point.y /= len(data.segments)
                
                self.init_markers(way_point.x, way_point.y)

                #return way_point.x, way_point.y
                return math.atan(way_point.y/way_point.x)


        def calc_velocity(self, angle):
                #max_angle is 20
                return (((20 + self.padding_angle) - angle) ** 2) * self.padding_velocity


        def main_pathing(self, data):
                #angle = self.pid.totalError(self.waypoint(data))
                #velocity = self.calc_velocity(angle)
                angle = (self.waypoint(data))*180/math.pi
                velocity = 1.1

                if self.dotest:
                    rospy.loginfo("angle = " + str(angle))
                    rospy.loginfo("velocity= " + str(velocity))

                #TODO: publisher
                drive_data = Twist()
                drive_data.angular.z = angle
                drive_data.linear.x = velocity

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

		if (self.count > 3):
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
