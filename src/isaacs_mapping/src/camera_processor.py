#!/usr/bin/env python
import rospy
import pyzed.defines as sl
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import cv2
import time
import FisheyeProcessor from "fisheye_processor"
import ZedProcessor from "zed_processor"


SETTINGS = {
    "fps" : 60
}

FISHEYE_SETTINGS = {
    "image_topic" : "fisheye_rgb",
	"camera_index" : 0
}

ZED_SETTINGS = {
    "resolution" : sl.PyRESOLUTION.PyRESOLUTION_HD720,
	"camera1_index" : 0,
    "camera1_topic_prefix" : "zed1",
    "camera2_index" : 1,
    "camera2_topic_prefix" : "zed2",
}

# read infomation from cameras
class CameraProcessor:

    #set up cameras and ros publisher
	def start(self):		
		rospy.init_node('camera_processer', anonymous=True)
		rospy.spin()

        self.fisheye_processor = FisheyeProcessor()
        self.fisheye_processor.initialize(FISHEYE_SETTINGS["camera_index"])
        self.zed1_processor = ZedProcessor()
        self.zed2_processor = ZedProcessor()
        self.zed1_processor.initialize(ZED_SETTINGS["camera1_index"], ZED_SETTINGS["camera1_topic_prefix"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
        self.zed2_processor.initialize(ZED_SETTINGS["camera2_index"], ZED_SETTINGS["camera2_topic_prefix"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
		self.process_camera(SETTINGS["fps"])

    def initialize_publisher(self):
        self.fisheye_image_publisher = rospy.Publisher(FISHEYE_SETTINGS["image_topic"], Image, queue_size = 20)

    #publishing images from cameras at each frame to a ros topic
	def process_camera(self, fps):
		fisheyeVid = cv2.VideoCapture(cameraIndex)
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
				fisheye_ret, self.fisheyeIm = self.fisheye_processor.get_info()
                self.zed1_processor.grab()
                self.zed2_processor.grab()
				self.image_publisher.publish()

			# the 'q' button is set as the quitting button
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# After the loop release the cap object
		fisheyeVid.release()

    #publish ros topics of the images info
	def publish_image(self):
		self.fisheye_image_publisher.publish(self.fisheyeIm)

if __name__ == '__main__':
	print('Node to publish cameras info start....')
	processor = CameraProcessor()
	processor.start()
