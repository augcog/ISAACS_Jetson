#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'marker_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ("no transform for this")
            rate.sleep()
            continue

        print(trans.transform.translation.y)
        
        #msg = geometry_msgs.msg.Twist()
        #msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        #msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        #turtle_vel.publish(msg)

        rate.sleep()
