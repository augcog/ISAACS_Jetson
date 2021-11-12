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
	
	def get_info(self) -> '[left, right, left_depth]':
        leftMat = core.PyMat()
        rightMat = core.PyMat()
        depthMat = core.PyMat()
        err = self.cam.grab(self.runtime)
        if err == tp.PyERROR_CODE.PySUCCESS:
            self.cam.retrieve_image(leftMat, sl.VIEW.LEFT)
            self.cam.retrieve_image(rightMat, sl.VIEW.RIGHT)
            self.cam.retrieve_image(depthMat, sl.VIEW.DEPTH)
        return [leftMat.get_data(), rightMat.get_data(), depthMat.get_data()]

	def close(self, im):
        self.cam.close()