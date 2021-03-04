#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import csv
import math
from zed_interfaces.srv import *

# w = csv.writer(open('datafile.csv', 'a'))
rpy_array = []
pose_is_set = False

def setZEDPose(pos_x, pos_y, pos_z, roll, pitch, yaw):
    try:
        set_zed_pose_service = rospy.ServiceProxy('/zed2/zed_node/set_pose', set_pose)
        ret = set_zed_pose_service(pos_x, pos_y, pos_z, roll, pitch, yaw) 
        return ret
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def callback(data):
    global pose_is_set

    if not pose_is_set:
        print("allen yang <3")
        roll_x, pitch_y, yaw_z = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
        pos_x = data.pose.position.x
        pos_y = data.pose.position.y
        pos_z = data.pose.position.z
        rpy_array.append([pos_x, pos_y, pos_z, roll_x, pitch_y, yaw_z])

        if len(rpy_array) >= 20:
            # average out the values and
            # call setPose service of zed with x, y, z, r, p, y
            last_arrays = rpy_array[-3:]
            s0, s1, s2, s3, s4, s5 = 0, 0, 0, 0, 0, 0  # sums
            for arr in last_arrays:
                s0 += arr[0]
                s1 += arr[1]
                s2 += arr[2]
                s3 += arr[3]
                s4 += arr[4]
                s5 += arr[5]
            denom = len(last_arrays)
            s0 /= denom
            s1 /= denom
            s2 /= denom
            s3 /= denom
            s4 /= denom
            s5 /= denom
            setZEDPose(s0, s1, s2, s3, s4, s5)

            start_mapping_service = rospy.ServiceProxy('/zed2/zed_node/start_3d_mapping', start_3d_mapping)
            ret = start_mapping_service(0.2, 8.0, 1.0) # resolution, range (meters), PC freq
            pose_is_set = True



def track_marker():
    print("Waiting for services...")
    rospy.wait_for_service('/zed2/zed_node/set_pose')
    rospy.wait_for_service('/zed2/zed_node/start_3d_mapping')
    print("Services Ready!")
    rospy.init_node('tracking_data', anonymous=True)
    rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)
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
