#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
import time

SETTINGS = {
	"fisheye_image_topic" : "fisheye_rgb",
	"fps" : 60,
	"camera_index" : 0
}

# read rgb images from a fisheye camera and publish it as a ros topic
class FisheyeProcessor:
	def start(self):		
		rospy.init_node('fisheye_processor', anonymous=True)
		self.image_publisher = rospy.Publisher(SETTINGS["fisheye_image_topic"], Image, queue_size = 20)
		rospy.spin()
		self.process_camera(SETTINGS["fps"], SETTINGS["camera_index"])

	def process_camera(self, fps, cameraIndex):
		vid = cv2.VideoCapture(cameraIndex)
		prevTime = time.time()
		timer = 0
		timeInterval = 1.0/fps
		while(True):
			currentTime = time.time()
			timer += (currentTime - prevTime)
			prevTime = currentTime 
			# Capture the video frame by frame
			if(timer >= timeInterval):
				timer -= timeInterval
				ret, frame = vid.read()
				self.image_publisher.publish(frame)

			# the 'q' button is set as the quitting button
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# After the loop release the cap object
		vid.release()


	def publish_image(self, im):
		self.image_publisher.publish(im)

if __name__ == '__main__':
	print('Node to publish fisheye info start....')
	processor = FisheyeProcessor()
	processor.start()
