#!/usr/bin/env python
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

SETTINGS = {
    "image_topic" : "rgb",
}
# read rgb images from a fisheye camera and publish it as a ros topic
class FisheyeProcessor:
	def initialize(self, cameraIndex, topic_prefix):
		self.video = cv2.VideoCapture(cameraIndex)
		self.initialize_publisher()
		self.topic_prefix = topic_prefix
	
	def grab(self) -> "dict":
		ret, frame = vid.read()
		data = dict()
		data["ret"] = ret
		data["rgb"] = frame
		self.publish_topics(data)
		return data

	#add prefix to the topic, since there are multiple zed cameras
    def generate_topic_name(self, topic):
        return self.topic_prefix + topic

    # create ros topics of the camera info
    def initialize_publisher(self):
		self.image_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["image_topic"]), Image, queue_size = 20)

    # publish ros topics of the camera info
    def publish_topics(self, data):
		self.image_publisher.publish(data["rgb"])

	def close(self):
		self.video.release()
