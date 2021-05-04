#!/usr/bin/python

import rospy, math
from std_msgs.msg import Int32MultiArray

class PID():
	def __init__(self, kp, ki, kd):
		self.Kp = kp
		self.Ki = ki
		self.Kd = kd
		self.p_error = 0.0
		self.i_error = 0.0
		self.d_error = 0.0

	def TotalError(self, cte):
		self.d_error = cte - self.p_error
		self.p_error = cte
		self.i_error += cte

		return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error

def callback(msg):
	global array
	array=msg.data

	

rospy.init_node('guide')

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
xycar_msg = Int32MultiArray()
rate= rospy.Rate(100)
array=[0,0,0,0,0,0,0,0]
pid=PID(0.25,0.0005,0.5)
a=0

while not rospy.is_shutdown():

	rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
	center=(array[6]+array[7])/2
	a=center-array[7]
	front=(array[0]-array[2])
	a1=front-array[2]	
	xycar_msg.data = [a-int(0.3*a1), 25]

	motor_pub.publish(xycar_msg)
	rate.sleep()
