#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import csv
import math
from zed_interfaces.srv import *
import cv2
import numpy as np
import tf


# w = csv.writer(open('datafile.csv', 'a'))
rpy_array = []
pose_is_set = False

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
        print("Service call failed: %s"%e)


def try_set_pose(cap):

	print("allen yang <3")
	camera_matrix = np.matrix([[1057.12, 0, 1131.19] ,
								[0, 1056.68, 614.561],
								[0, 0, 1]])	
	dist_coeffs = np.array([-0.0411877, 0.00998836, 0.00035581, 0.000354927, -0.00480993])

	print("camera_matrix: " + str(camera_matrix))
	print("dist_coeffs: " + str(dist_coeffs))
	

	# Capture frame-by-frame, take only left camera image
	ret, frame = cap.read()
	#frame = np.split(frame, 2, axis=1)[0]
	cv2.imshow('zed', frame)

	# Our operations on the frame come here
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
	parameters =  cv2.aruco.DetectorParameters_create()

	#lists of ids and the corners belonging to each id
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	markerLength = 0.16

	rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)

	# we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
	rotation_matrix = np.array([[0, 0, 0, 0],
				[0, 0, 0, 0],
				[0, 0, 0, 0],
				[0, 0, 0, 1]],
								        dtype=float)
	rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

	# convert the matrix to a quaternion
	quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

	# To visualize in rviz, you can, for example, publish a PoseStamped message:

	rpy_array.append([tvec[0], tvec[1], tvec[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
     
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



def track_marker():
	global pose_is_set
	
	print("Waiting for services...")
	#rospy.wait_for_service('/zed2/zed_node/set_pose')
	#rospy.wait_for_service('/zed2/zed_node/start_3d_mapping')
	print("Services Ready!")
	#rospy.init_node('tracking_data', anonymous=True)
    #rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)

	cap = cv2.VideoCapture(0, cv2.CAP_V4L)

	
	while not pose_is_set:
		try_set_pose(cap)
	
	rospy.spin()

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
