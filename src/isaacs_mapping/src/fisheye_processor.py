#!/usr/bin/env python
import cv2
#import rospy
import numpy as np
#from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge, CvBridgeError 
#from sensor_msgs.msg import Image

SETTINGS = {
    #"image_topic" : "rgb",
}
# read rgb images from a fisheye camera and publish it as a ros topic
class FisheyeProcessor:
	def initialize(self, cameraIndex):#, topic_prefix):
		#self.video = cv2.VideoCapture(cameraIndex)
		self.video = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink' , cv2.CAP_GSTREAMER)
		#self.bridge = bridge = CvBridge()
		#self.topic_prefix = topic_prefix
		#self.initialize_publisher()

	def grab(self):
		ret, frame = self.video.read()
		data = dict()
		data["ret"] = ret
		data["rgb"] = frame
		#self.publish_topics(data)
		return data

	'''#add prefix to the topic, since there are multiple zed cameras
	def generate_topic_name(self, topic):
		return self.topic_prefix + topic

	# create ros topics of the camera info
	def initialize_publisher(self):
		self.image_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["image_topic"]), Image, queue_size = 20)

	# publish ros topics of the camera info
	def publish_topics(self, data):
		if data["rgb"] is not None:
			data["rgb"] = np.uint8(data["rgb"])
			image_message = self.bridge.cv2_to_imgmsg(data["rgb"], encoding="passthrough")
			self.image_publisher.publish(image_message)'''

	def close(self):
		self.video.release()
