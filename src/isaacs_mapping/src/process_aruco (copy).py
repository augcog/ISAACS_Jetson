#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import csv
import math
from zed_interfaces.srv import *
import cv2
import numpy as np
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

import sys

# w = csv.writer(open('datafile.csv', 'a'))
rpy_array = []
pose_is_set = False
pub_new_tf = []
tfBuffer = []
mesh_trans = []

def setZEDPose(pos_x, pos_y, pos_z, o_x, o_y, o_z, o_w):
	global pose_is_set
	global tfBuffer
	global mesh_trans
	#roll, pitch, yaw = euler_from_quaternion(o_x, o_y, o_z, o_w)
	#try:
	#print('Set camera pose with tvec: ' + str([pos_x, pos_y, pos_z]) + ', rvec: ' + str([o_x, o_y, o_z, o_w]))
    #set_zed_pose_service = rospy.ServiceProxy('/zed2/zed_node/set_pose', set_pose)
	#reset_zed_odom_service = rospy.ServiceProxy('/zed2/zed_node/reset_odometry', reset_odometry)
	#reset_zed_tracking_service = rospy.ServiceProxy('/zed2/zed_node/reset_tracking', reset_tracking)
	#ret = reset_zed_odom_service()
	#ret = reset_zed_tracking_service()

	#ret = set_zed_pose_service(pos_x, pos_y, pos_z, roll, pitch, yaw)
	#ret = reset_zed_odom_service()

	trans = 0.0
    # get transform from marker to map, and set mesh to it
	try:
		trans = tfBuffer.lookup_transform('fid701','map', rospy.Time(0), rospy.Duration(1.0))                                       
		# Sets the marker--> map transform of the mesh frame
		t = geometry_msgs.msg.TransformStamped()
		t.header.frame_id = "map"
		t.child_frame_id = "mesh"
		t.header.stamp = rospy.Time.now()
		t.transform = trans.transform
		mesh_trans = t # save transform to be republished continously
		tfm = tf2_msgs.msg.TFMessage([t])
		pub_new_tf.publish(tfm)
		pose_is_set = True
		start_mapping()
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print("UH OH, no transform between marker and map??")
	return
	#except rospy.ServiceException as e:
	#	print("Service call failed: %s"%e)

def start_mapping():
    try:
        start_mapping_service = rospy.ServiceProxy('/zed2/zed_node/start_3d_mapping', start_3d_mapping)
        ret = start_mapping_service(0.02, 8.0, 1.0) # resolution, range (meters), PC freq
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def callback(data):
	global pose_is_set
	if pose_is_set: return
	print("allen yang <3")

	quaternion = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
	rotation_matrix = quaternion_rotation_matrix(quaternion)
	M = np.array([[0, 0, 0, data.pose.position.x],
				[0, 0, 0, data.pose.position.y],
				[0, 0, 0, data.pose.position.z],
				[0, 0, 0, 1]],
				dtype=float)
	M[:3, :3] = rotation_matrix
	inv_M = np.linalg.inv(M)

	quaternion = tf.transformations.quaternion_from_matrix(inv_M)
	translation = [inv_M[0][3], inv_M[1][3], inv_M[2][3]]

	rpy_array.append([translation[0], translation[1], translation[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
     
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
		#print('tvec: ' + str([s0, s1, s2]) +'\n' + 'rvec: ' + str([s3, s4, s5, s6]) +'\n' )
		setZEDPose(s0, s1, s2, s3, s4, s5, s6)




def track_marker():
    global pose_is_set
    global pub_new_tf
    global tfBuffer

    print("Waiting for services...")
    #rospy.wait_for_service('/zed2/zed_node/set_pose')
    rospy.wait_for_service('/zed2/zed_node/start_3d_mapping')
    print("Services Ready!")
    rospy.init_node('tracking_data', anonymous=True)
    rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)
    pub_new_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #rate = rospy.Rate(10.0)
    map_mesh_trans_found = False
    while not rospy.is_shutdown() and not map_mesh_trans_found:
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "mesh"
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 1.0

        t.transform.rotation.x = 0.508
        t.transform.rotation.y = 0.340
        t.transform.rotation.z = 0.640
        t.transform.rotation.w = 0.466

        tfm = tf2_msgs.msg.TFMessage([t])
        pub_new_tf.publish(tfm)

        try:
            
            trans = tfBuffer.lookup_transform('map','mesh', rospy.Time(0), rospy.Duration(2))
            map_mesh_trans_found = True
            print("found")
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("looking for mesh map trans")
            continue
    while not pose_is_set:
            print("setting pose")
            setZEDPose(0,0,0,0,0,0,0) 
    rate = rospy.Rate(10.0)    
    while not rospy.is_shutdown() and mesh_trans:
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "mesh"
        t.header.stamp = rospy.Time.now()
        t.transform = mesh_trans.transform
	    #mesh_trans = t
	    #tfm = tf2_msgs.msg.TFMessage([t])
	    #pub_new_tf.publish(tfm)
	    #mesh_trans.stamp = rospy.Time.now() #update time
        tfm = tf2_msgs.msg.TFMessage([t])
        pub_new_tf.publish(tfm)
        rate.sleep()

    rospy.spin()

# source: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

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
