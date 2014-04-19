#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import numpy as np
import pdb

#

class ros_service():
	def __init__(self):
		cv2.namedWindow("ros-color",1)
		cv2.namedWindow("ros-depth",1)
		cv2.setMouseCallback("ros-depth",self.onmouse)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/color",Image,self.callbackCamera)
		self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_rect",Image,self.callbackDepth)
	def onmouse(self,event,x,y,flags,param):
		global mouse_x
		global mouse_y
		mouse_x = x
		mouse_y = y
	def callbackCamera(self,data):
		try:
			#pdb.set_trace()
			color_im = self.bridge.imgmsg_to_cv(data)
			color_image = np.array(color_im)	
			cv2.imshow("ros-color",color_image)
			cv2.waitKey(10)
		except CvBridgeError, e:
			print e
	def callbackDepth(self,data):
		global mouse_x
		global mouse_y
		try:
			DEPTH_IMAGE_MM_TO_IN = .0393
			DEPTH_IMAGE_METER_TO_GRAY = 1.0/3500.0
			#pdb.set_trace()
			self.cv_depth = self.bridge.imgmsg_to_cv(data)
			
			depth_image = np.array(self.cv_depth)#*np.ones((480,640))
			d = depth_image[int(mouse_y),int(mouse_x)]*DEPTH_IMAGE_MM_TO_IN
			depth_image = DEPTH_IMAGE_METER_TO_GRAY*depth_image
			#depth_image = depth_image.astype(int)
			#gray_image = 255*(depth_image*DEPTH_IMAGE_METERS_TO_GRAY)
			#depth_image = cv2.convertScaleAbs(depth_image)
			print d	
			cv2.imshow("ros-depth",depth_image)
			cv2.waitKey(10)
		except CvBridgeError, e:
			print e

def mainloop():
	initvariables()
	rospy.init_node('listener',anonymous=True)
	rc = ros_service()
	rate = rospy.Rate(10.0)
	rospy.sleep(1)
	while not rospy.is_shutdown():
		rospy.sleep(0.001)
def initvariables():
	global mouse_x
	global mouse_y
	mouse_x = 0
	mouse_y = 0

if __name__ == '__main__':
    mainloop()
