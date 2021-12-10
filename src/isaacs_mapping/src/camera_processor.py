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

    #set up cameras
	def start(self):	
		self.fisheye_processor = FisheyeProcessor()
		self.fisheye_processor.initialize(FISHEYE_SETTINGS["camera1_index"])#, FISHEYE_SETTINGS["camera1_topic_prefix"])

		self.zed1_processor = ZedProcessor()
		self.zed1_processor.initialize(ZED_SETTINGS["camera1_serial_number"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
	       
		self.zed2_processor = ZedProcessor()
		self.zed2_processor.initialize(ZED_SETTINGS["camera2_serial_number"], ZED_SETTINGS["resolution"], SETTINGS["fps"])
			
		self.process_camera(SETTINGS["fps"])

    #grab images from camera
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
				self.fisheye_processor.grab()
				self.zed1_processor.grab()
				self.zed2_processor.grab()

			# the 'q' button is set as the quitting button
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# After the loop release the cap object
		self.fisheye_processor.close()
		self.zed1_processor.close()
		self.zed2_processor.close()
		cv2.destroyAllWindows()
		print("Cameras closed")

	
	""" HAVE NOT TESTED 
	Detect the marker in the image and return the opencv camera to marker transformation matrix."""
	def detect_marker(self, image):
		#Set the params for marker and camera
		self.aruco_dict = cv2.aruco.DICT_5X5_250
		self.marker_size = 0.105
		self.camera_intrinsic =  np.matrix([[519.5537109375, 0.0, 656.6468505859375], 
							[0.0, 519.5537109375, 363.6219482421875], 
							[0.0, 0.0, 1.0]])
		self.camera_dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

		frame = image
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dictionary = cv2.aruco.Dictionary_get(self.aruco_dict)
		aruco_parameters =  cv2.aruco.DetectorParameters_create()

		# lists of ids and the corners belonging to each id
		corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary=aruco_dictionary, parameters = aruco_parameters)
		if ids is None:
			return None
		rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_intrinsic, self.camera_dist_coeffs)
		print("Rvec", rvec)
		print("Tvec", tvec)
		rvec = rvec[0][0]
		print("rvec[00]", rvec)
		tvec = tvec[0][0]
		print("tvec[00]", tvec)

		# visualize image of marker being detected
		if self.show_marker_UI:
			cv2.aruco.drawAxis(frame, self.camera_intrinsic, self.camera_dist_coeffs, rvec, tvec, self.marker_size)
			cv2.imshow('image',frame) 
			cv2.waitKey(1)

		rotation_matrix = np.identity(4)
		rmat = cv2.Rodrigues(rvec)[0]
		rotation_matrix[:3, :3] = rmat  
		rotation_matrix[:3, 3] = tvec
		print("Rotation Matrix (marker in cam coords)")
		print(rotation_matrix)
		return np.linalg.inv(rotation_matrix) #opencv camera coordinate system to marker coordinate system, this camera coordinate system is not zed camera coord

if __name__ == '__main__':
	print('Node to publish cameras info start....')
	cameras = sl.Camera.get_device_list()
	#helper code to show serial number of all connected zed cameras 
	#for cam in cameras:
	#	print(cam.serial_number)
	processor = CameraProcessor()
	processor.start()
