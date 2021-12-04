#!/usr/bin/env python
import pyzed.sl as sl
import cv2
import time
from fisheye_processor import FisheyeProcessor 
from zed_processor import ZedProcessor 


SETTINGS = {
    "fps" : 30
}

FISHEYE_SETTINGS = {
    "camera1_topic_prefix" : "fisheye1",
    "camera1_index" : 0
}

ZED_SETTINGS = {
    "resolution" : sl.RESOLUTION.HD720,
    "camera1_serial_number" : 20919438,
    "camera1_topic_prefix" : "zed1",

    "camera2_serial_number" : 21974856,
    "camera2_topic_prefix" : "zed2",
}

# read infomation from cameras
class CameraProcessor:

    #set up cameras and ros publisher
	def start(self):	
		#self.fisheye_processor = FisheyeProcessor()
		#self.fisheye_processor.initialize(FISHEYE_SETTINGS["camera1_index"])#, FISHEYE_SETTINGS["camera1_topic_prefix"])

		self.zed1_processor = ZedProcessor()
		self.zed1_processor.initialize(ZED_SETTINGS["camera1_serial_number"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
	       
		#self.zed2_processor = ZedProcessor()
		#self.zed2_processor.initialize(ZED_SETTINGS["camera2_serial_number"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
			
		self.process_camera(SETTINGS["fps"])
		rospy.spin()

    #publishing images from cameras at each frame to a ros topic
	def process_camera(self, fps):
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
				#self.fisheye_processor.grab()
				self.zed1_processor.grab()
		        	#self.zed2_processor.grab()

			# the 'q' button is set as the quitting button
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# After the loop release the cap object
		#self.fisheye_processor.close()
		self.zed1_processor.close()
		#self.zed2_processor.close()
		cv2.destroyAllWindows()

if __name__ == '__main__':
	print('Node to publish cameras info start....')
	cameras = sl.Camera.get_device_list()
	#helper code to show serial number of all connected zed cameras 
	#for cam in cameras:
	#	print(cam.serial_number)
	processor = CameraProcessor()
	processor.start()
