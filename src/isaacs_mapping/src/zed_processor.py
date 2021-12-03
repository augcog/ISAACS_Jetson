#!/usr/bin/env python
import cv2
import pyzed.sl as sl

#from sensor_msgs.msg import CompressedImage
#from sensor_msgs.msg import Image
#from geometry_msgs.msg import Vector3
#from geometry_msgs.msg import Quaternion


SETTINGS = {
    #"left_image_topic" : "left_rgb",
    #"right_image_topic" : "right_rgb",
    #"left_depth_topic" : "left_depth",
    #"position_topic" : "position",
    #"rotation_topic" : "rotation",
}

# read rgb images from a zed camera and publish it as a ros topic
class ZedProcessor:
	#def initialize(self, cameraIndex, topic_prefix, resolution, fps):
    def initialize(self, serialNumber, resolution, fps):
        init = sl.InitParameters()
        init.camera_resolution = resolution
        init.camera_fps = fps
        init.set_from_serial_number(serialNumber)
        self.cam = sl.Camera()
        #self.topic_prefix = topic_prefix
        #self.initialize_publisher()

        if not self.cam.is_opened():
            print("Opening ZED Camera " + str(serialNumber) + "...")
        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

        self.runtime = sl.RuntimeParameters()
        tracking_parameters = sl.PositionalTrackingParameters()
        err = self.cam.enable_positional_tracking(tracking_parameters)
        print(err)
	
    def grab(self):
        leftMat = sl.Mat()
        rightMat = sl.Mat()
        depthMat = sl.Mat()
        zed_pose = sl.Pose()
        err = self.cam.grab(self.runtime)
        data = dict()
        if err == sl.ERROR_CODE.SUCCESS:
            self.cam.retrieve_image(leftMat, sl.VIEW.LEFT)
            data["left_rgb"] = leftMat
            self.cam.retrieve_image(rightMat, sl.VIEW.RIGHT)
            data["right_rgb"] = rightMat
            self.cam.retrieve_image(depthMat, sl.VIEW.DEPTH)
            data["left_depth"] = depthMat
            # Get the pose of the camera relative to the world frame
            state = self.cam.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            # translation
            py_translation = sl.Translation()
            data["position"] = zed_pose.get_translation(py_translation).get()
            # orientation quaternion
            py_orientation = sl.Orientation()
            data["rotation"] = zed_pose.get_orientation(py_orientation).get()
            #self.publish_topics(data)
            print(data)
        return data

    '''#add prefix to the topic, since there are multiple zed cameras
    def generate_topic_name(self, topic):
        return self.topic_prefix + topic

    # create ros topics of the camera info
    def initialize_publisher(self):
        self.left_image_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["left_image_topic"]), Image, queue_size = 20)
        self.right_image_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["right_image_topic"]), Image, queue_size = 20)
        self.left_depth_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["left_depth_topic"]), Image, queue_size = 20)
        self.position_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["position_topic"]), Vector3, queue_size = 20)
        self.rotation_publisher = rospy.Publisher(self.generate_topic_name(SETTINGS["rotation_topic"]), Quaternion, queue_size = 20)

    # publish ros topics of the camera info
    def publish_topics(self, data):
        self.left_image_publisher.publish(data["left_rgb"])
        self.right_image_publisher.publish(data["right_rgb"])
        self.left_depth_publisher.publish(data["left_depth"])
        self.position_publisher.publish(data["position"])
        self.rotation_publisher.publish(data["rotation"])
    '''

    def close(self, im):
        self.cam.close()
