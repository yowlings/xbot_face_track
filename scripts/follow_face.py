#!/usr/bin/env python
#coding=utf-8

"""
For tracking and move to the face that appered in the eyes of xbot.
Copyright (c) 2016 Peng Wang (Rocwang).  All rights reserved.
This program is free software; you can redistribute or modify it.
More details about xbot robot platform is available in http://wiki.ros.org/Robots/Xbot.
"""

import rospy, sys, termios, tty, math, time
from geometry_msgs.msg import Twist
from xbot_msgs.msg import XbotState
from xbot_msgs.msg import face_pose

class FollowFace():
	"""docstring for FollowFace"""
	def __init__(self):
		self.cmd = Twist()
		self.alfa = 0.1
		self.state = 0
		self.upordown = 1 #0 is down and 1 is up

		self.help = """









		"""
		self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		self.platform_pub = rospy.Publisher('/mobile_base/commands/other_motors', XbotState, queue_size = 1)
		rospy.Subscriber("face_in_camera", face_pose, self.face_callback)
		rospy.Subscriber("/mobile_base/xbot/state", XbotState, self.state_callback)
		# rospy.sleep(0.1)
		rospy.spin()







	def face_callback(self, pose):
		self.state = 1
		motor_cmd = XbotState()
		motor_cmd.height_percent = 0
		motor_cmd.camera_degree = 90
		motor_cmd.platform_degree = 90
		self.platform_pub.publish(motor_cmd)
		self.image_width = pose.image_width
		self.image_height = pose.image_height
		self.x_left = int(0.4*self.image_width)
		self.x_right = int(0.6*self.image_width)
		self.diff_x = pose.face_x - self.image_width/2

		# if pose.face_x > self.x_right:
		# 	self.diff_x = pose.face_x - self.x_right
		# elif pose.face_x < self.x_left:
		# 	self.diff_x = pose.face_x - self.x_left
		# else:
		# 	self.diff_x = 0
		self.cmd.linear.x = 0#0.15*pose.face_z/100
		self.cmd.angular.z = self.alfa*self.diff_x*math.pi/300
		print self.cmd.angular.z

		try:
			self.cmd_pub.publish(self.cmd)
		except Exception, e:
			raise e
		finally:
			self.cmd_pub.publish(self.cmd)

	def state_callback(self,xbot_state):
		# time.sleep(0.1)
		if self.state == 0:
			print 'searching face...'
			# print xbot_state
			try:
				motor_cmd = XbotState()
				motor_cmd.height_percent = xbot_state.height_percent
				motor_cmd.camera_degree = xbot_state.camera_degree
				current_degree = xbot_state.platform_degree
				if self.upordown == 1:
					# print current_degree
					goal_degree = current_degree + 1 #current_degree + 6
					if goal_degree > 180:
						self.upordown = 0
						goal_degree = 360 - goal_degree
				elif self.upordown == 0:
					goal_degree = current_degree - 1
					if goal_degree < 0:
						self.upordown = 1
						goal_degree = -goal_degree
				motor_cmd.platform_degree = goal_degree
				self.platform_pub.publish(motor_cmd)



			except Exception, e:
				raise e
			finally:
				self.platform_pub.publish(motor_cmd)




if __name__ == '__main__':
	rospy.init_node('follow_face')
	try:
		rospy.loginfo('initialization system for follow_face...')
		FollowFace()
		print 'process follow_face done and quit.'
	except rospy.ROSInterruptException:
		rospy.loginfo('node follow_face termindated.')






