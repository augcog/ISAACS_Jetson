#!/usr/bin/env python
import cv2

# read rgb images from a fisheye camera and publish it as a ros topic
class FisheyeProcessor:
	def initialize(self, cameraIndex):
		self.video = cv2.VideoCapture(cameraIndex)
	
	def get_info(self) -> "[ret, frame]":
		ret, frame = vid.read()
		return [ret, frame]

	def close(self, im):
		self.video.release()
