#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

class RobotMover():
	def __init__(self, pub, v_speed, a_speed):
		self.pub = pub
		self.r = rospy.Rate(10)
		self.v_speed = v_speed
		self.a_speed = a_speed
	
	def move_linear(self, x_val):
		total_time = abs(x_val) / self.v_speed
		iterations = int(math.floor(total_time * 10)) #10hz
		if(x_val > 0):
			speed_command = self.v_speed
		else:
			speed_command = -1*self.v_speed
		for i in range(iterations):
			self.move(speed_command, 0)
			self.r.sleep()
		self.move(0,0)
		self.r.sleep()

	def move_angular(self, z_val):
		z_val = z_val * math.pi #put in radians
		total_time = abs(z_val) / self.a_speed
		iterations = int(math.floor(total_time * 10)) #10hz
		if(z_val > 0):
			speed_command = self.a_speed
		else:
			speed_command = -1*self.a_speed
		for i in range(iterations):
			self.move(0,speed_command)
			self.r.sleep()
		self.move(0,0)
		self.r.sleep()

	def move(self, x_val, z_val):
		twist = Twist()
		twist.linear.x = x_val
		twist.angular.z = z_val
		self.pub.publish(twist)

def main():
	rospy.init_node('robot_mover')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	#set linear speed in meters/sec and angular speed in radians/sec
	mover = RobotMover(pub, 0.5, 0.5)
	#rotate 0.5 radians
	mover.move_angular(0.5)
	#move forward 2 meters
	mover.move_linear(2.0)

if __name__ == '__main__':
	main()
