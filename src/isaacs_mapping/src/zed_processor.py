#!/usr/bin/env python
import cv2
import pyzed.camera as zcam
import pyzed.types as tp
import pyzed.core as core
import pyzed.defines as sl

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
    def initialize(self, cameraIndex, resolution, fps):
        init = zcam.PyInitParameters()
        init.camera_resolution = resolution
        init.camera_linux_id = cameraIndex
        init.camera_fps = fps
        self.cam = zcam.PyZEDCamera()
        #self.topic_prefix = topic_prefix
        #self.initialize_publisher()

        if not cam.is_opened():
            print("Opening ZED Camera " + string(cameraIndex) + "...")
        status = cam.open(init)
        if status != tp.PyERROR_CODE.PySUCCESS:
            print(repr(status))
            exit()

        self.runtime = zcam.PyRuntimeParameters()
        tracking_parameters = sl.PositionalTrackingParameters()
        err = zed.enable_positional_tracking(tracking_parameters)
        print(err)
	
	def grab(self) -> "dict":
        leftMat = core.PyMat()
        rightMat = core.PyMat()
        depthMat = core.PyMat()
        zed_pose = sl.Pose()
        err = self.cam.grab(self.runtime)
        data = dict()
        if err == tp.PyERROR_CODE.PySUCCESS:
            self.cam.retrieve_image(leftMat, sl.VIEW.LEFT)
            data["left_rgb"] = leftMat
            self.cam.retrieve_image(rightMat, sl.VIEW.RIGHT)
            data["right_rgb"] = rightMat
            self.cam.retrieve_image(depthMat, sl.VIEW.DEPTH)
            data["left_depth"] = depthMat
            # Get the pose of the camera relative to the world frame
            state = self.cam.get_position(zed_pose, sl.REFERENCE_FRAME.FRAME_WORLD)
            # translation
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            data["position"] = py_translation
            # orientation quaternion
            py_orientation = sl.Orientation()
            ox = round(zed_pose.get_orientation(py_orientation).get()[0], 3)
            oy = round(zed_pose.get_orientation(py_orientation).get()[1], 3)
            oz = round(zed_pose.get_orientation(py_orientation).get()[2], 3)
            ow = round(zed_pose.get_orientation(py_orientation).get()[3], 3)
            data["rotation"] = py_orientation
            #self.publish_topics(data)
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