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
    "camera1_left_image_topic" : "zed1_left_rgb",
    "camera1_right_image_topic" : "zed1_right_rgb",
    "camera1_left_depth_topic" : "zed1_left_depth",
    "camera1_position_topic" : "zed1_position",
    "camera1_rotation_topic" : "zed1_rotation",

    "camera2_index" : 1,
    "camera2_left_image_topic" : "zed2_left_rgb",
    "camera2_right_image_topic" : "zed2_right_rgb",
    "camera2_left_depth_topic" : "zed2_left_depth",
    "camera2_position_topic" : "zed2_position",
    "camera2_rotation_topic" : "zed2_rotation"
}

# read infomation from cameras
class CameraProcessor:

    #set up cameras and ros publisher
	def start(self):		
		rospy.init_node('camera_processer', anonymous=True)
        self.initialize_publisher()

		rospy.spin()

        self.fisheye_processor = FisheyeProcessor()
        self.zed1_processor = ZedProcessor()
        self.zed2_processor = ZedProcessor()
        self.fisheye_processor.initialize(FISHEYE_SETTINGS["camera_index"])
        self.zed1_processor.initialize(ZED_SETTINGS["camera1_index"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
        self.zed2_processor.initialize(ZED_SETTINGS["camera2_index"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
		self.process_camera(SETTINGS["fps"])

    def initialize_publisher(self):
        self.fisheye_image_publisher = rospy.Publisher(FISHEYE_SETTINGS["image_topic"], Image, queue_size = 20)
        self.zed1_left_image_publisher = rospy.Publisher(ZED_SETTINGS["camera1_left_image_topic"], Image, queue_size = 20)
        self.zed1_right_image_publisher = rospy.Publisher(ZED_SETTINGS["camera1_right_image_topic"], Image, queue_size = 20)
        self.zed1_left_depth_publisher = rospy.Publisher(ZED_SETTINGS["camera1_left_depth_topic"], Image, queue_size = 20)
        self.zed1_position_publisher = rospy.Publisher(ZED_SETTINGS["camera1_position_topic"], Vector3, queue_size = 20)
        self.zed1_rotation_publisher = rospy.Publisher(ZED_SETTINGS["camera1_rotation_topic"], Quaternion, queue_size = 20)

        self.zed2_left_image_publisher = rospy.Publisher(ZED_SETTINGS["camera2_left_image_topic"], Image, queue_size = 20)
        self.zed2_right_image_publisher = rospy.Publisher(ZED_SETTINGS["camera2_right_image_topic"], Image, queue_size = 20)
        self.zed2_left_depth_publisher = rospy.Publisher(ZED_SETTINGS["camera2_left_depth_topic"], Image, queue_size = 20)
        self.zed2_position_publisher = rospy.Publisher(ZED_SETTINGS["camera2_position_topic"], Vector3, queue_size = 20)
        self.zed2_rotation_publisher = rospy.Publisher(ZED_SETTINGS["camera2_rotation_topic"], Quaternion, queue_size = 20)

    #publishing images from cameras at each frame to a ros topic
	def process_camera(self, fps):
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
				fisheye_ret, self.fisheyeIm = self.fisheye_processor.get_info()
                self.zed1_left, self.zed1_right, self.zed1_depth, self.zed1_pos, self.zed1_rotation = self.zed1_processor.get_info()
                self.zed2_left, self.zed2_right, self.zed2_depth, self.zed2_pos, self.zed2_rotation = self.zed2_processor.get_info()
				self.image_publisher.publish()

			# the 'q' button is set as the quitting button
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# After the loop release the cap object
		vid.release()

    #publish ros topics of the images info
	def publish_image(self):
		self.fisheye_image_publisher.publish(self.fisheyeIm)
        self.zed1_left_image_publisher.publish(self.zed1_left)
        self.zed1_right_image_publisher.publish(self.zed1_right)
        self.zed1_left_depth_publisher.publish(self.zed1_depth)
        self.zed1_position_publisher.publish(self.zed1_pos)
        self.zed1_rotation_publisher.publish(self.zed1_rotation)

        self.zed2_left_image_publisher.publish(self.zed2_left)
        self.zed2_right_image_publisher.publish(self.zed2_right)
        self.zed2_left_depth_publisher.publish(self.zed2_depth)
        self.zed2_position_publisher.publish(self.zed2_pos)
        self.zed2_rotation_publisher.publish(self.zed2_rotation)

if __name__ == '__main__':
	print('Node to publish cameras info start....')
	processor = CameraProcessor()
	processor.start()
