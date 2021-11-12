#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
import time
import FisheyeProcessor from "fisheye_processor"


SETTINGS = {
    "fps" : 60
}

FISHEYE_SETTINGS = {
    "fisheye_image_topic" : "fisheye_rgb",
	"fisheye_camera_index" : 0
}

# read infomation from cameras
class CameraProcessor:
	def start(self):		
		rospy.init_node('camera_processer', anonymous=True)
        self.fisheye_image_publisher = rospy.Publisher(FISHEYE_SETTINGS["fisheye_image_topic"], Image, queue_size = 20)


		rospy.spin()

        self.fisheye_processor = FisheyeProcessor()
        self.fisheye_processor.initialize(FISHEYE_SETTINGS["camera_index"])
		self.process_camera(SETTINGS["fps"])

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
				ret, frame = vid.read()
				self.image_publisher.publish(frame)

			# the 'q' button is set as the quitting button
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# After the loop release the cap object
		vid.release()


	def publish_image(self, im):
		self.fisheye_image_publisher.publish(im)

if __name__ == '__main__':
	print('Node to publish cameras info start....')
	processor = CameraProcessor()
	processor.start()
