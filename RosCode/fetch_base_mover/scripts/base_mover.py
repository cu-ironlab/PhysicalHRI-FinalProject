#!/usr/bin/env python

import rospy
import math
import time
import copy
import sys
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class RobotMover():
	def __init__(self, pub, v_speed, a_speed):
		self.pub = pub
		self.r = rospy.Rate(10)
		self.v_speed = v_speed
		self.a_speed = a_speed
		self.pose = [0,0]
		self.angle = [0,0,0,1]
	
	def move_linear(self, x_val):
		if(x_val > 0):
			speed_command = self.v_speed
		else:
			speed_command = -1*self.v_speed
		pose_start = copy.deepcopy(self.pose)
		dist_moved = 0
		while(dist_moved < (abs(x_val) - self.v_speed/10.0)):
			self.move(speed_command, 0)
			self.r.sleep()
			dist_moved = math.sqrt((pose_start[0]-self.pose[0])**2 + (pose_start[1] - self.pose[1])**2)
		self.move(0,0)
		self.r.sleep()

	def move_angular(self, z_val):
		if(abs(z_val) > 1):
			print("WARNING: rotation greater than 1 radian requested. Trimming to 1 radian")
			z_val = math.copysign(1, z_val)
		z_val = z_val * math.pi #put in radians
		if(z_val > 0):
			speed_command = self.a_speed
		else:
			speed_command = -1*self.a_speed
		rot = Rotation.from_quat(self.angle)
		last_angle = rot.as_euler('xyz')
		angle_dist = 0
		while(angle_dist < (abs(z_val) - self.a_speed/10.0)):
			self.move(0,speed_command)
			self.r.sleep()
			rot = Rotation.from_quat(self.angle)
			new_angle = rot.as_euler('xyz')
			new_angle_dist = abs(abs(new_angle[2]) - abs(last_angle[2]))
			angle_dist += new_angle_dist
			last_angle = new_angle
		self.move(0,0)
		self.r.sleep()

	def move(self, x_val, z_val):
		twist = Twist()
		twist.linear.x = x_val
		twist.angular.z = z_val
		self.pub.publish(twist)

	def odom_update(self, odom_msg):
		pose = odom_msg.transform
		self.pose =[pose.translation.x, pose.translation.y]
		self.angle = [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]

	def print_state(self):
		print("Position: [x: "+str(self.pose[0])+", y: "+str(self.pose[1])+"]")
		rot = Rotation.from_quat(self.angle)
		angle = rot.as_euler('xyz')
		print("Angle: [x: "+str(angle[0])+", y: "+str(angle[1])+", z: "+str(angle[2]))


def main():
	if(len(sys.argv) < 3):
		print("USAGE: base_mover.py <angular movement> <linear movement>")
		return
	rospy.init_node('robot_mover')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	#set linear speed in meters/sec and angular speed in radians/sec
	mover = RobotMover(pub, 0.2, 0.5)
	sub = rospy.Subscriber('/vicon/fetch/fetch', TransformStamped, callback=mover.odom_update)
	
	time.sleep(0.1)
	#rotate 
	mover.move_angular(float(sys.argv[1]))
	#move forward
	mover.move_linear(float(sys.argv[2]))

if __name__ == '__main__':
	main()
