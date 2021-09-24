#!/usr/bin/env python
import rospy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from isaacs_mapping.msg import Matrix4x4
import csv
import math
from zed_interfaces.srv import *
import cv2
import numpy as np
from math import sqrt
import tf


SETTINGS = {
#	"marker_size" : 0.08, # length of one side of the marker, in meters
       "marker_size" : 0.105,        
#        "marker_size" : 0.160,
#	"aruco_dict" : cv2.aruco.DICT_ARUCO_ORIGINAL, # which dictionary the marker is in  
	"aruco_dict" : cv2.aruco.DICT_5X5_250, # which dictionary the marker is in 

        # For 720p mode
        "camera_intrinsic" : np.matrix([[519.5537109375, 0.0, 656.6468505859375], 
							[0.0, 519.5537109375, 363.6219482421875], 
							[0.0, 0.0, 1.0]]),
        #"camera_intrinsic_rectified" : np.matrix([[1.0, 0.0, 0.0],
                                                        #[0.0, 1.0, 0.0],
                                                        #[0.0, 0.0, 1.0]]), # delete this
	"camera_dist_coeffs" : np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
	"camera_extrinsic" : [[1, 0, 0, 0.06],
					   [0, 1, 0, 0],
					   [0, 0, 1, 0],
					   [0, 0, 0, 1]],
	"converted_point_cloud_node" : "converted_cloud", # name of the node we publish our converted point clouds to
	"converted_mesh_node" : "converted_mesh",
	"conversion_matrix_node" : "zed2marker_transform"
}

class PointCloudCamToMarkerConverter:
	def __init__(self, show_marker_UI = False, image_is_compressed = False, print_cam_location = False):
		self.marker_size = SETTINGS["marker_size"]
		self.aruco_dict = SETTINGS["aruco_dict"]
		self.camera_intrinsic = SETTINGS["camera_intrinsic"]
		self.camera_dist_coeffs = SETTINGS["camera_dist_coeffs"]
		self.camera_extrinsic = SETTINGS["camera_extrinsic"]

		self.pose_is_set = False
		self.image_subscriber = None
		self.pose_subscriber = None
		self.pointcloud_subscriber = None
		self.pointcloud_publisher = None
		self.mesh_subscriber = None
		self.mesh_publisher = None
		self.converted_point_cloud_node = SETTINGS["converted_point_cloud_node"]
		self.converted_mesh_node = SETTINGS["converted_mesh_node"]
		self.conversion_matrix_node = SETTINGS["conversion_matrix_node"]
		self.camera_pose = None
		self.zed2marker = [[1, 0, 0, 0], # the zed2marker conversion matrix
					    [0, 1, 0, 0], 
						[0, 0, 1, 0], 
						[0, 0, 0, 1]]
		
		self.calibrate_initial_pos = False # currently we assume camera is at its origin when detects the marker
		self.print_camera_pos = print_cam_location
		self.image_compressed = image_is_compressed
		self.show_marker_UI = show_marker_UI


	def start(self):		
		rospy.init_node('tracking_data', anonymous=True)
		# create the subscribers needed for detecting marker and set pose
		if self.image_compressed:
			self.image_subscriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color/compressed', CompressedImage, self.process_image)
		else:
			self.image_subscriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.process_image)
		self.pose_subscriber = rospy.Subscriber('/zed2/zed_node/pose', PoseStamped, self.update_camera_pose)
		#self.pointcloud_publisher = rospy.Publisher(self.converted_point_cloud_node, PointCloud2, queue_size = 20)
		#self.mesh_publisher = rospy.Publisher(self.converted_mesh_node, Mesh, queue_size = 20)


		self.transform_matrix_publisher = rospy.Publisher(self.conversion_matrix_node, Matrix4x4, queue_size = 10)

		rospy.spin()

	""" Called when receiving an image message from ZED. Uses the image to detect the marker and set the pose."""
	def process_image(self, image_data):
		if self.pose_is_set:
			return # nitzan
		# deserialize the image so it can be used for OpenCV ArUco marker detection
		if self.image_compressed:
			image_serialized = np.fromstring(image_data.data, np.uint8)
			image_deserialized = cv2.imdecode(image_serialized, cv2.IMREAD_COLOR)
		else:
			image_deserialized = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
		
		if self.try_set_pose(image_deserialized):
			print("finish set pose")
			self.pose_is_set = True
			# after the pose is set, update the subscriber and keep/create the ones needed for converting point clouds
			#self.image_subscriber.unregister() # uncommented -- nitzan
			if not self.print_camera_pos:
				self.pose_subscriber.unregister()
			#self.pointcloud_subscriber = rospy.Subscriber('/zed2/zed_node/mapping/fused_cloud', PointCloud2, self.convert_zed_pose)
			#self.mesh_subscriber = rospy.Subscriber('/voxblox_node/mesh', Mesh, self.convert_voxblox_mesh, queue_size = 20)
			#self.pointcloud_subscriber = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.convert_zed_pose, queue_size = 20)

			#publish the matrix
			self.publish_zed_transform()

	def publish_zed_transform(self):
		'''
		conversion_matrix_data = Float32MultiArray()  # the data to be sent, initialise the array
		conversion_matrix_data.data = []
		for a in self.zed2marker:
			conversion_matrix_data.data += list(a) # assign the array with the value you want to send
		'''

		'''
		#translations, rotations = self.transform_from_matrix(self.zed2marker) # incorrect conversion!
                r = R.from_dcm(self.zed2marker[:3, :3])
                q = r.as_quat() # x y z w
                print(q)
                tf_msg = Transform()


		#tf_msg.translation.x = translations[0]
		#tf_msg.translation.y = translations[1]
		#tf_msg.translation.z = translations[2]

                # translation vector is simply the last column of the 4x4 augmented rotation matrix
		tf_msg.translation.x = self.zed2marker[0][3]
		tf_msg.translation.y = self.zed2marker[1][3]
		tf_msg.translation.z = self.zed2marker[2][3]
		
                tf_msg.rotation.x = q[0]
		tf_msg.rotation.y = q[1]
		tf_msg.rotation.z = q[2]
		tf_msg.rotation.w = q[3]

		self.transform_matrix_publisher.publish(tf_msg)

        
                x = tf_msg.rotation.x # nitzan
                y = tf_msg.rotation.y
                z = tf_msg.rotation.z
                w = tf_msg.rotation.w
                print("X:", x, "Y:", y, "Z:", z, "W:", w)
                euler_r, euler_p, euler_y = self.euler_from_quaternion(x, y, z, w) # nitzan
                deg_r = euler_r * 180 / math.pi
                deg_p = euler_p * 180 / math.pi
                deg_y = euler_y * 180 / math.pi
                print("R:", deg_r, "P:", deg_p, "Y:", deg_y)

                deg_r, deg_p, deg_y = r.as_euler('xyz', degrees=True)
                print("R:", deg_r, "P:", deg_p, "Y:", deg_y)
		'''
		tf_msg = Matrix4x4()
		tf_msg.row1 = self.zed2marker[0]
		tf_msg.row2 = self.zed2marker[1]
		tf_msg.row3 = self.zed2marker[2]
		tf_msg.row4 = self.zed2marker[3]
		self.transform_matrix_publisher.publish(tf_msg)
		print("transform published")

	"""Called when receiving a pose message from Zed. Store the pose to keep self.camera_pose update to date."""
	def update_camera_pose(self, data):
		self.camera_pose = data.pose
		# print out the camera's position in the world coordinate system 
		if self.print_camera_pos and self.pose_is_set:
			converted_camera_pose = np.array([self.camera_pose.position.x, self.camera_pose.position.y, self.camera_pose.position.z, 1])
			print("camera world pos:", np.matmul(self.zed2marker , converted_camera_pose))

	""" Called when receiving a point cloud message from ZED. It converts point cloud positions to the marker coordinate system, 
	and publishes the converted point clouds to a new ROS node."""
        def convert_zed_pose(self, pointcloud_data):
            if not self.pose_is_set:
	        return
                #pointcloud_data = self.convert_point_clouds(pointcloud_data)
	        #self.pointcloud_publisher.publish(pointcloud_data) 

	'''
	""" Helper function that loops through all the point clouds and converts their position to the marker coordinate system. """
	def convert_point_clouds(self, pointcloud_data):
		reader = point_cloud2.read_points(pointcloud_data, field_names = ["x", "y", "z", "rgb"], skip_nans=True)
		new_points = []
		print("Start converting point clouds.")
		for p in reader:
			# transfer point to aruco marker's corrdinate system
			new_p = self.convert_point_to_marker(p[0], p[1], p[2])
			new_p.append(p[3]) #add rgb value to the point
			new_points.append(new_p)
		print("Converted point clouds message of size " , len(new_points))
		return point_cloud2.create_cloud(pointcloud_data.header, pointcloud_data.fields, new_points)

	def convert_voxblox_mesh(self, meshdata):
		if(len(meshdata.mesh_blocks) <= 0):
			return 
		num = 0
		print("Converting voxblox mesh to marker coordinate system.")
		for block in meshdata.mesh_blocks:
			num += len(block.x)
			block.x = list(block.x)
			block.y = list(block.y)
			block.z = list(block.z)
			for i in range(len(block.x)):
				converted_p = self.voxblox_pos_to_float_pos(block.x[i], block.y[i], block.z[i], block.index, meshdata.block_edge_length)
				converted_p = self.convert_point_to_marker(converted_p[0], converted_p[1], converted_p[2])
				converted_p = self.float_pos_to_voxblox_pos(converted_p[0], converted_p[1], converted_p[2], block.index, meshdata.block_edge_length)
				#block.x[i] = converted_p[0]
				#block.y[i] = converted_p[1]
				#block.z[i] = converted_p[2]
		self.mesh_publisher.publish(meshdata)
		print("Finished converting mesh with vertices size:", num)
		

	def voxblox_pos_to_float_pos(self, x, y, z, index, block_edge_length):
		new_x = ((float)(x / 32768.0) + index[0]) * block_edge_length
		new_y = ((float)(y / 32768.0) + index[1]) * block_edge_length
		new_z = ((float)(z / 32768.0) + index[2]) * block_edge_length
		return [new_x, new_y, new_z]

	def float_pos_to_voxblox_pos(self, x, y, z, index, block_edge_length):
		new_x = np.uint16((x/block_edge_length - index[0]) * 32768)
		new_y = np.uint16((y/block_edge_length - index[1]) * 32768)
		new_z = np.uint16((z/block_edge_length - index[2]) * 32768)
		return [new_x, new_y, new_z]

	def convert_point_to_marker(self, x, y, z):
		converted_p = list(np.matmul(self.zed2marker, [x, y, z, 1]))
		converted_p = list([converted_p[0], converted_p[1], converted_p[2]]/converted_p[3])
		return converted_p
	'''
	
	""" Get an image as input and trys to calculate the zed to marker transformation matrix"""
	def try_set_pose(self,image):				
		opencv_cam2marker = self.detect_marker(image)
		if(opencv_cam2marker is None):
			return False
                                

                zed2cam = np.array([[0, -1, 0, 0], # zed camera coordinate system to opencv camera coordinate system transform
							[0, 0, -1, 0],
							[1, 0, 0, 0],
							[0, 0, 0, 1]])	

		self.zed2marker = np.matmul(opencv_cam2marker, zed2cam)
                print("rotation matrix (unity coords)")
                print(self.zed2marker)
		return True

	""" Detect the marker in the image and return the opencv camera to marker transformation matrix."""
	def detect_marker(self, image):
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
                #rvec[0] -= 3.141592
               # rvec[0] = rvec[0] % 3.14592
               # theta = math.sqrt(rvec[0]**2 + rvec[1]**2 + rvec[2]**2)
               # rvec[0] /= theta
                #rvec[0] *= (180 / math.pi)
               # rvec[1] *= (180 / math.pi)
               # rvec[2] *= (180 / math.pi)
		tvec = tvec[0][0]
                print("tvec[00]", tvec)
                #print("R:", rvec, "T:", tvec)
		# visualize image of marker being detected
		if self.show_marker_UI:
			cv2.aruco.drawAxis(frame, self.camera_intrinsic, self.camera_dist_coeffs, rvec, tvec, self.marker_size)
                        #cv2.aruco.drawDetectedMarkers(gray, corners)
			cv2.imshow('image',frame) 
                        #cv2.imshow('gimage',gray)
			cv2.waitKey(1)
			#cv2.destroyAllWindows()

		rotation_matrix = np.identity(4)
		rmat = cv2.Rodrigues(rvec)[0]
		rotation_matrix[:3, :3] = rmat  
		rotation_matrix[:3, 3] = tvec
                print("Rotation Matrix (marker in cam coords)")
                print(rotation_matrix)
		return np.linalg.inv(rotation_matrix) #opencv camera coordinate system to marker coordinate system, this camera is not zed camera


	# source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
	@staticmethod
	def euler_from_quaternion(x, y, z, w):
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

        # INCORRECT! Do NOT use this function. Instead, Use Scipy 1.2.3 (the most recent Scipy version for Python 2.7)
	# Matrix to quaternion source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	@staticmethod
	def transform_from_matrix(mat):
                #determinant = np.linalg.det(mat)
                #print(determinant)
            
                # DO NOT USE this function "transform_from_matrix". See comments above

                """
		Convert a homography matrix into a translation and a rotation
		Translation is represented as [x, y, z]
		Rotation is represented as a quaternion [x, y, z, w]
		"""
		x_translation = mat[0][3]
		y_translation = mat[1][3]
		z_translation = mat[2][3]

		w_rotation = sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2])
		x_rotation = (mat[2][1] - mat[1][2]) / (4 * w_rotation)
		y_rotation = (mat[0][2] - mat[2][0]) / (4 * w_rotation)
		z_rotation = (mat[1][0] - mat[0][1]) / (4 * w_rotation)
                return # DO NOT USE THIS FUNCTION.
		#return [x_translation, y_translation, z_translation], [x_rotation, y_rotation, z_rotation, w_rotation]



if __name__ == '__main__':
	print('Node to write tracked data started .......')
	converter = PointCloudCamToMarkerConverter(image_is_compressed = False)
	converter.start()
