#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math

if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')
    pub_new_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
    
    while not rospy.is_shutdown():
        #br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        rospy.sleep(0.1)


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

        #br.sendTransform(t)
        tfm = tf2_msgs.msg.TFMessage([t])
        pub_new_tf.publish(tfm)
        
