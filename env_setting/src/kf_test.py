#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import message_filters

from sensor_msgs.msg import CompressedImage
from fiducial_msgs.msg import FiducialArray


dt = 1 #????

kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
	                                 [0, 1, 0, 0]], np.float32)

kalman.transitionMatrix = np.array([[1, 0, dt, 0],
	                                [0, 1, 0, dt], 
	                                [0, 0, 1, 0],
	                                [0, 0 ,0 ,1]], np.float32)

kalman.processNoiseCov = np.array([[1, 0, 0, 0],
	                               [0, 1, 0, 0], 
	                               [0, 0, 1, 0],
	                               [0, 0 ,0 ,1]], np.float32) * 0.03

measurement = np.array((2, 1), np.float32)
prediction = np.zeros((2, 1), np.float32)


class KF:
	def __init__(self):
		self.prediction = None
		self.pts_center = np.array([0, 0])


def center(points):
	x = np.float32((points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4.0)
	y = np.float32((points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4.0)
	return np.array([np.float32(x), np.float32(y)], np.float32)


def callback_img(data):
	print("\n-------------------------------------------")
	np_arr		= np.fromstring(data.data, np.uint8)
	image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	prediction = kalman.predict()
	print(prediction)

	cv2.circle(image, (kf.pts_center[0], kf.pts_center[1]), 5, (0, 0, 255), -1)
	cv2.circle(image, (prediction[0], prediction[1]), 3, (0, 255, 0), -1)
	cv2.imshow("image", image)
	cv2.waitKey(1)


def callback_ver(data):
	fids = data.fiducials
	fids_num = len(fids)

	for i in fids:
		pts = np.array([[i.x0, i.y0], [i.x1, i.y1], [i.x2, i.y2], [i.x3, i.y3]])
		pts = np.int0(pts)
		kf.pts_center = center(pts)
		print(kf.pts_center)
		kalman.correct(kf.pts_center)



if __name__ == "__main__":	
	try :
		kf = KF()
		rospy.init_node("kf_test")
		rospy.Subscriber("/fiducial_vertices", FiducialArray, callback_ver, queue_size=10)
		rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback_img, queue_size=10)

		rospy.spin()

	except rospy.ROSInterruptException:
		pass