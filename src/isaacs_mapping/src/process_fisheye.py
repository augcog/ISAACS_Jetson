#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
import numpy as np

SETTINGS = {
	"fisheye_image_topic" : "fisheye_rgb",
}

# read rgb images from a fisheye camera and publish it as a ros topic
class FisheyeProcessor:
	def start(self):		
		rospy.init_node('fisheye_processor', anonymous=True)
		self.image_publisher = rospy.Publisher(SETTINGS["fisheye_image_topic"], Image, queue_size = 20)
		rospy.spin()


	def publish_image(self, im):
		self.image_publisher.publish(im)



if __name__ == '__main__':
	print('Node to publish fisheye info start....')
	processor = FisheyeProcessor()
	processor.start()
