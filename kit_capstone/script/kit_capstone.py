#!/usr/bin/env python

import rospy

from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def totalError(self, cte):
        self.d_error = cte - self.p_errer
        self.p_error = cte
        self.i_error = += cte

        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error


class CapstoneMainDrive:
	def __init__(self):
                # Parameter
                kp = rospy.get_param("/kp", 0.0)
                ki = rospy.get_param("/ki", 0.0)
                kd = rospy.get_param("/kd", 0.0)

                self.dotest = rospy.get_param("/dotest", 0)
                self.padding_angle = rospy.get_param("/padding_angle")
                self.padding_velocity = rospy.get_param("/padding_velocity")

                self.pid = PID(kp, ki, kd)

                # init
		rospy.init_node('kit_capstone', anonymous=True)

		self.lidar_sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_callback) 
		#self.imu_sub = rospy.Subscriber('kit_capstone_imu', , ) 
		#self.steer_pub 


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
                #way_point.y /= len(data.segments)

                return way_point.x, way_point.y


        def calc_velocity(self, angle):
                return (((max_angle + self.padding_angle) - angle) ** 2) * self.padding_velocity


         def main_pathing(self, data):
                angle = self.pid.totalError(self.waypoint(data))
                velocity = self.calc_velocity(angle)

                if self.dotest:
                    rospy.loginfo("angle = " , angle)
                    rospy.loginfo("velocity= " , velocity)



if __name__ == '__main__':
	try:
		capstone_drive = CapstoneMainDrive()
		rospy.spin()

	except rospy.ROSInterruptException:
		print(error)
		pass
