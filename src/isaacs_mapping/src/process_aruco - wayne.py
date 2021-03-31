#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs import point_cloud2
import csv
import math
from zed_interfaces.srv import *
import cv2
import numpy as np
import tf


# w = csv.writer(open('datafile.csv', 'a'))
rpy_array = []
R_array = []
pose_is_set = False
image_subsriber = None
publisher = None
conversion_matrix = [[1, 0, 0, 0], 
					[0, 1, 0, 0], 
					[0, 0, 1, 0], 
					[0, 0, 0, 1]]

'''
def setZEDPose(pos_x, pos_y, pos_z, o_x, o_y, o_z, o_w):
    global pose_is_set
    roll, pitch, yaw = eupter_from_quaternion(o_x, o_y, o_z, o_w)
    try:
        print('Setting camera pose with tvec: ' + str([s0, s1, s2]) + ', rvec: ' + str([s3, s4, s5, s6]))
        set_zed_pose_service = rospy.ServiceProxy('/zed2/zed_node/set_pose', set_pose)
        ret = set_zed_pose_service(pos_x, pos_y, pos_z, roll, pitch, yaw)
        print("Pose set: " + str(ret))
        pose_is_set = True
        return ret
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)'''


def convert_zed_pose(pointcloud_data):
	if(not pose_is_set):
		return 
	global publisher
	print("Im called")
	pointcloud_data = convert_point_clouds(pointcloud_data)
	publisher.publish(pointcloud_data) 

def convert_point_clouds(pointcloud_data):
	global conversion_matrix
	reader = point_cloud2.read_points(pointcloud_data, skip_nans=True)
	new_points = []
	for p in reader:
		#transfer point to aruco marker's corrdinate system
		new_p = list(np.matmul(conversion_matrix, [p[0], p[1], p[2], 1]))
		print(new_p[0:3] + [list(p)[3:]])
		new_points.append(new_p[0:3] + list(p)[3:])
	return point_cloud2.create_cloud(pointcloud_data.header, pointcloud_data.fields, new_points)

def try_set_pose(image):

	#print("allen yang <3")
	camera_matrix = np.matrix([[528.85, 0, 648.825] ,
								[0, 528.49, 363.0625],
								[0, 0, 1]])	
	dist_coeffs = np.array([-0.0445958, 0.0145996, -0.00654037, -0.000450176, 0.000393205])

	#print("camera_matrix: " + str(camera_matrix))
	#print("dist_coeffs: " + str(dist_coeffs))
	

	# Capture frame-by-frame, take only left camera image
	#ret, frame = cap.read()
	frame = image
	#frame = np.split(frame, 2, axis=1)[0]
	#cv2.imshow('zed', frame)
	# Our operations on the frame come here
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
	parameters =  cv2.aruco.DetectorParameters_create()

	#lists of ids and the corners belonging to each id
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	markerLength = 0.16

	rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
	if(rvec is None or tvec is None):
		return False
	print("rvec not None here:", rvec)
	# we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
	rotation_matrix = np.array([[0, 0, 0, tvec[0]],
				[0, 0, 0, tvec[1]],
				[0, 0, 0, tvec[2]],
				[0, 0, 0, 1]],
								        dtype=float)
	rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
	R = np.linalg.inv(rotation_matrix)
	R_array.append(R)

	#get average R
	global conversion_matrix
	if len(R_array) >= 20: 
		R_sum = R_array[0]
		for i in range(1, len(R_array)):
			R_sum += R_array[i]
		conversion_matrix = R_sum / len(len(R_array))
		return True
	return False
'''
	# convert the matrix to a quaternion
	quaternion = tf.transformations.quaternion_from_matrix(R)

	# To visualize in rviz, you can, for example, publish a PoseStamped message:
	rpy_array.append([R[0][3], R[1][3], R[2][3], quaternion[0], quaternion[1], quaternion[2], quaternion[3]])

	if len(rpy_array) >= 20:
		last_arrays = rpy_array[-3:]
		s0, s1, s2, s3, s4, s5, s6 = 0, 0, 0, 0, 0, 0, 0  # sums
		for arr in last_arrays:
			s0 += arr[0]
			s1 += arr[1]
			s2 += arr[2]
			s3 += arr[3]
			s4 += arr[4]
			s5 += arr[5]
			s6 += arr[5]

		denom = len(last_arrays)
		s0 /= denom
		s1 /= denom
		s2 /= denom
		s3 /= denom
		s4 /= denom
		s5 /= denom
		s6 /= denom
		
 		print('Setting camera pose with tvec: ' + str([s0, s1, s2]) + ', rvec: ' + str([s3, s4, s5, s6]))
		#setZEDPose(s0, s1, s2, s3, s4, s5, s6)
		#start_mapping_service = rospy.ServiceProxy('/zed2/zed_node/start_3d_mapping', start_3d_mapping)
		#ret = start_mapping_service(0.2, 8.0, 1.0) # resolution, range (meters), PC freq
'''

def process_image(image_data):
	image_serialized = np.fromstring(image_data.data, np.uint8)#np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
	image_deserialized = cv2.imdecode(image_serialized, cv2.IMREAD_COLOR)
	print("getting image")
	global pose_is_set
	global image_subsriber
	if(pose_is_set):
		return
	if(try_set_pose(image_deserialized)):
		print("finish set pose")
		pose_is_set = True
		image_subsriber.unregister()
		rospy.Subscriber('/zed2/zed_node/mapping/fused_cloud', PointCloud2, convert_zed_pose)


def track_marker():
	global pose_is_set
	global publisher
	global image_subsriber
	print("Waiting for services...")
	#rospy.wait_for_service('/zed2/zed_node/set_pose')
	#rospy.wait_for_service('/zed2/zed_node/start_3d_mapping')
	print("Services Ready!")
	
	rospy.init_node('tracking_data', anonymous=True)
	image_subsriber = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color/compressed', CompressedImage, process_image)
	publisher = rospy.Publisher('converted_cloud', PointCloud2, queue_size=10)
	
	rospy.spin()
	#pose_is_set = True
	
	#cap = cv2.VideoCapture(0, cv2.CAP_V4L)
	
	#while not pose_is_set:
	#	try_set_pose(cap)
	

# source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
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

if __name__ == '__main__':
    print('Node to write tracked data started .......')
    # w.writerow(['seq', 'time_stamp', 'frame_id', 'pos_x', 'pos_y', 'pos_z', 'roll_x', 'pitch_y', 'yaw_z'])
    track_marker()
