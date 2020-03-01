from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import math


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24

rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

   time.sleep(0.0)
   image = frame.array

   
   cv2.imshow("Frame",image)
   key = cv2.waitKey(1) & 0xFF
   rawCapture.truncate(0)
   
   if key == ord("q"):
       cv2.destroyAllWindows()
       break
