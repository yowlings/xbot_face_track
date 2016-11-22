#!/usr/bin/env python
#coding=utf-8

"""
This program detects the faces appeared in the eyes of xbot and publishes the position of faces to the topic 'face_in_camera'.
Copyright (c) 2016 Peng Wang (Rocwang).  All rights reserved.
This program is free software; you can redistribute or modify it.
More details about xbot robot platform is available in http://wiki.ros.org/Robots/Xbot.
"""

import rospy, sys, termios, tty, math, time, cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from xbot_msgs.msg import face_pose
# import roslib
# roslib.load_manifest('my_package')
from cv_bridge import CvBridge, CvBridgeError




class FaceDetect():
	"""docstring for FollowFace"""
	def __init__(self):

		self.help = """









		"""
		if rospy.has_param("~cascPath"):
			self.cascPath = rospy.get_param("~cascPath")
		else:
			rospy.set_param("~cascPath","../scripts/haarcascade_frontalface_default.xml")

		self.faceCascade = cv2.CascadeClassifier(self.cascPath)
		self.face_pose = face_pose()
		self.bridge = CvBridge()
		self.face_pub = rospy.Publisher("face_in_camera", face_pose, queue_size = 1)
		rospy.Subscriber("/camera/rgb/image_raw", Image, self.detected_callback)
		rospy.spin()





	def detected_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			faces = self.faceCascade.detectMultiScale(
				gray,
				scaleFactor=1.1,
				minNeighbors=5,
				minSize=(30,30)
			)
			print faces

			for (x, y, w, h) in faces:
				cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
			cv2.imshow("Image window", cv_image)
			cv2.waitKey(30)
			(rows,cols,channels)=cv_image.shape
			print rows, cols,channels
			if len(faces)>0:
				self.face_pose.image_width = rows
				self.face_pose.image_height = cols
				self.face_pose.face_x = faces[0][0]+faces[0][2]*0.5
				self.face_pose.face_y = faces[0][1]+faces[0][3]*0.5
				self.face_pose.face_z = 1000
				self.face_pub.publish(self.face_pose)
		except CvBridgeError as e:
			print(e)






if __name__ == '__main__':
	rospy.init_node('detect_face')
	try:
		rospy.loginfo('initialization system for detect_face...')
		FaceDetect()
		print 'process detecting_face done and quit.'
	except rospy.ROSInterruptException:
		rospy.loginfo('node follow_face termindated.')

