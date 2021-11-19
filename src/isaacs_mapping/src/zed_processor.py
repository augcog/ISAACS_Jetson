#!/usr/bin/env python
import cv2
import pyzed.camera as zcam
import pyzed.types as tp
import pyzed.core as core
import pyzed.defines as sl

# read rgb images from a zed camera and publish it as a ros topic
class ZedProcessor:
	def initialize(self, cameraIndex, resolution, fps):
        init = zcam.PyInitParameters()
        init.camera_resolution = resolution
        init.camera_linux_id = cameraIndex
        init.camera_fps = fps
        self.cam = zcam.PyZEDCamera()

        if not cam.is_opened():
            print("Opening ZED Camera " + string(cameraIndex) + "...")
        status = cam.open(init)
        if status != tp.PyERROR_CODE.PySUCCESS:
            print(repr(status))
            exit()
        self.runtime = zcam.PyRuntimeParameters()

        tracking_parameters = sl.PositionalTrackingParameters()
        err = zed.enable_positional_tracking(tracking_parameters)
	
	def get_info(self) -> '[left, right, left_depth]':
        leftMat = core.PyMat()
        rightMat = core.PyMat()
        depthMat = core.PyMat()
        zed_pose = sl.Pose()
        err = self.cam.grab(self.runtime)
        if err == tp.PyERROR_CODE.PySUCCESS:
            self.cam.retrieve_image(leftMat, sl.VIEW.LEFT)
            self.cam.retrieve_image(rightMat, sl.VIEW.RIGHT)
            self.cam.retrieve_image(depthMat, sl.VIEW.DEPTH)
            # Get the pose of the camera relative to the world frame
            state = self.cam.get_position(zed_pose, sl.REFERENCE_FRAME.FRAME_WORLD)
            # translation
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            # orientation quaternion
            py_orientation = sl.Orientation()
            ox = round(zed_pose.get_orientation(py_orientation).get()[0], 3)
            oy = round(zed_pose.get_orientation(py_orientation).get()[1], 3)
            oz = round(zed_pose.get_orientation(py_orientation).get()[2], 3)
            ow = round(zed_pose.get_orientation(py_orientation).get()[3], 3)
        return [leftMat.get_data(), rightMat.get_data(), depthMat.get_data(), py_translation, py_orientation]

	def close(self, im):
        self.cam.close()