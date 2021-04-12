#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2
import csv
import math
from zed_interfaces.srv import *
import cv2
import numpy as np
import tf


SETTINGS = {
	"marker_size" : 0.05, #length of one side of the marker, in meters
	"aruco_dict" : cv2.aruco.DICT_ARUCO_ORIGINAL, #which dictionary the marker is in 
    "camera_intrinsic" : np.matrix([[519.5537109375, 0.0, 656.6468505859375], 
									[0.0, 519.5537109375, 363.6219482421875], 
									[0.0, 0.0, 1.0]]),
	"camera_dist_coeffs" : np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
	"camera_extrinsic" : [[1, 0, 0, 0.06],
						[0, 1, 0, 0],
						[0, 0, 1, 0],
						[0, 0, 0, 1]],
	"converted_point_cloud_node" : "converted_cloud" #name of the node we publish our converted point clouds to
}

class PointCloudCamToMarkerConverter:
	def __init__(self, show_marker_UI = False, image_is_compressed = False, print_cam_location = False):
		self.marker_size = SETTINGS["marker_size"]
		self.aruco_dict = SETTINGS["aruco_dict"]
		self.camera_intrinsic = SETTINGS["camera_intrinsic"]
		self.camera_dist_coeffs = SETTINGS["camera_dist_coeffs"]
		self.camera_extrinsic = SETTINGS["camera_extrinsic"]

		self.rpy_array = []
		self.R_array = []
		self.pose_is_set = False
		self.image_subsriber = None
		self.pose_subsriber = None
		self.pointcloud_subsriber = None
		self.pointcloud_publisher = None
		self.converted_point_cloud_node = SETTINGS["converted_point_cloud_node"]
		self.camera_pose = None
		self.conversion_matrix = [[1, 0, 0, 0], 
								[0, 1, 0, 0], 
								[0, 0, 1, 0], 
								[0, 0, 0, 1]]
		
		self.calibrate_initial_pos = False
		self.print_camera_pos = print_cam_location
		self.image_compressed = image_is_compressed
		self.show_marker_UI = show_marker_UI

	def track_marker(self):		
		rospy.init_node('tracking_data', anonymous=True)
		if(self.image_compressed):
			self.image_subsriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color/compressed', CompressedImage, self.process_image)
		else:
			self.image_subsriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.process_image)
		self.pose_subsriber = rospy.Subscriber('/zed2/zed_node/pose', PoseStamped, self.update_camera_pose)
		self.pointcloud_publisher = rospy.Publisher(self.converted_point_cloud_node, PointCloud2, queue_size=10)
		rospy.spin()

	def process_image(self, image_data):
		if(self.image_compressed):
			image_serialized = np.fromstring(image_data.data, np.uint8)
			image_deserialized = cv2.imdecode(image_serialized, cv2.IMREAD_COLOR)
		else:
			image_deserialized = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)

		if(self.pose_is_set):
			return

		if(self.try_set_pose(image_deserialized)):
			print("finish set pose")
			self.pose_is_set = True
			self.image_subsriber.unregister()
			if(not self.print_camera_pos):
				self.pose_subsriber.unregister()
			self.pointcloud_subsriber = rospy.Subscriber('/zed2/zed_node/mapping/fused_cloud', PointCloud2, self.convert_zed_pose)

	def update_camera_pose(self, data):
		self.camera_pose = data.pose
		#print out camera's position in world coordinate system 
		if(self.print_camera_pos and self.pose_is_set):
			converted_camera_pose = np.array([self.camera_pose.position.x, self.camera_pose.position.y, self.camera_pose.position.z, 1])
			print("camera world pos:", np.matmul(self.conversion_matrix , converted_camera_pose))

	def convert_zed_pose(self, pointcloud_data):
		if(not self.pose_is_set):
			return 
		pointcloud_data = self.convert_point_clouds(pointcloud_data)
		self.pointcloud_publisher.publish(pointcloud_data) 

	def convert_point_clouds(self, pointcloud_data):
		reader = point_cloud2.read_points(pointcloud_data, field_names = ["x", "y", "z", "rgb"], skip_nans=True)
		new_points = []
		for p in reader:
			#transfer point to aruco marker's corrdinate system
			new_p = list(np.matmul(self.conversion_matrix, [p[0], p[1], p[2], 1]))
			new_p = list([new_p[0], new_p[1], new_p[2]]/new_p[3])
			new_p.append(p[3])
			new_points.append(new_p)
		return point_cloud2.create_cloud(pointcloud_data.header, pointcloud_data.fields, new_points)

	def try_set_pose(self,image):				
		frame = image

		# Our operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dictionary = cv2.aruco.Dictionary_get(self.aruco_dict)
		aruco_parameters =  cv2.aruco.DetectorParameters_create()

		#lists of ids and the corners belonging to each id
		corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary=aruco_dictionary, parameters = aruco_parameters)
		if(ids is None):
			return False
		rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_intrinsic, self.camera_dist_coeffs)
		rvec = rvec[0][0]
		tvec = tvec[0][0]

		#visualize image of marker being detected
		if(self.show_marker_UI):
			cv2.aruco.drawAxis(gray, self.camera_intrinsic, self.camera_dist_coeffs, rvec, tvec, self.marker_size)
			cv2.aruco.drawDetectedMarkers(gray, corners)
			cv2.imshow('image',gray) 
			cv2.waitKey(0)
			cv2.destroyAllWindows()

		# we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
		rotation_matrix = np.identity(4)
		rmat = cv2.Rodrigues(rvec)[0]
		rotation_matrix[:3, :3] = rmat #marker in camera coordinate system 
		rotation_matrix[:3, 3] = tvec
		cam2marker = np.linalg.inv(rotation_matrix)
		
		zed2cam = np.array([[0, -1, 0, 0],
							[0, 0, -1, 0],
							[1, 0, 0, 0],
							[0, 0, 0, 1]])	
		if(self.calibrate_initial_pos):
			#when we detect marker camera is not at (0,0,0), need to take camera's pose into account
			camera_R = np.array([[0, 0, 0, self.camera_pose.position.x],
					[0, 0, 0, self.camera_pose.position.y],
					[0, 0, 0, self.camera_pose.position.z],
					[0, 0, 0, 1]],
									dtype=float)
			camera_rvec = euler_from_quaternion(camera_pose.orientation.x, camera_pose.orientation.y, 
												camera_pose.orientation.z, camera_pose.orientation.w)
			camera_R[:3, :3], _ = cv2.Rodrigues(camera_rvec)
			camera_R = np.linalg.inv(camera_R)
			camera_R = np.matmul(camera_R, self.camera_extrinsic)
			self.conversion_matrix = np.matmul(np.matmul(cam2marker, zed2cam), camera_R)
		else:
			self.conversion_matrix = np.matmul(cam2marker, zed2cam)
		'''
		R_array.append(np.matmul(cam2marker, zed2cam))

		if len(R_array) >= 20: 
			R_sum = R_array[0]
			for i in range(1, len(R_array)):
				print(i, R_array[i])
				R_sum += R_array[i]
			conversion_matrix = R_sum / len(R_array)
			return True
		return False
		'''
		return True

	# source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
	def euler_from_quaternion(self, x, y, z, w):
			"""
			Convert a quaternion into euler angles (roll, pitch, yaw)
			roll is rotation around x in radians (counterclockwise)
			pitch is rotation around y in radians (counterclockwise)
			yaw is rotation around z in radians (counterclockwise)
			"""
			t0 = +2.0 * (w * x + y * z)
			t1 = +1.0 - 2.0 * (x * x + y * y)
			roll_x = math.atan2(t0, t1)
		
			t2 = +2.0 * (w * y - z * x)
			t2 = +1.0 if t2 > +1.0 else t2
			t2 = -1.0 if t2 < -1.0 else t2
			pitch_y = math.asin(t2)
		
			t3 = +2.0 * (w * z + x * y)
			t4 = +1.0 - 2.0 * (y * y + z * z)
			yaw_z = math.atan2(t3, t4)
		
			return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
	print('Node to write tracked data started .......')
	converter = PointCloudCamToMarkerConverter()
	converter.track_marker()
