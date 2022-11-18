#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import codecs
from GPS_preprocessing import coord_transform, nmea_parsing
import rospy
import time
import numpy as np
from pykalman import KalmanFilter
from std_msgs.msg import String, Float32
from mi_msgs.msg import GPS
import csv    
from scipy import linalg

class Gps:

	def __init__(self):
		self.port = '/dev/ttyACM0'
		self.baud = 115200
		self.gps_cmd = GPS()
		self.gps_pub = rospy.Publisher('gps_messages', GPS, queue_size = 10)
		self.parsing_check = False
		self.ser = serial.Serial(self.port, self.baud, timeout=0)	
		self.GPS_core = []
		self.check_kalman = []

	def kalman_df(self,tm):
		global kf
		global x_now
		global P_now
		if len(self.check_kalman) == 0:
			self.check_kalman.append(tm)
		elif len(self.check_kalman) >= 1 and len(self.check_kalman) <2 :
			self.check_kalman.append(tm)
			measurements = np.asarray(self.check_kalman)
			initial_state_mean = [measurements[0,0], 0, measurements[0,1], 0]
			transition_matrix = [[1, 1, 0, 0],
							[0, 1, 0, 0],
							[0, 0, 1, 1],
							[0, 0, 0, 1]]
			observation_matrix = [[1, 0, 0, 0],
							[0, 0, 1, 0]]

			kf = KalmanFilter(transition_matrices = transition_matrix,
						observation_matrices = observation_matrix,
						initial_state_mean = initial_state_mean,
						em_vars=['transition_covariance', 'initial_state_covariance'])

			kf = kf.em(measurements)
			(filtered_state_means, filtered_state_covariances) = kf.filter(measurements)
			x_now = filtered_state_means[-1, :]
			P_now = filtered_state_covariances[-1, :]
		elif len(self.check_kalman) >= 2:
			(x_now, P_now) = kf.filter_update(filtered_state_mean = x_now, filtered_state_covariance = P_now, observation = tm)
			kalman_tm = [x_now[0], x_now[2]]
			return kalman_tm
	
	def parser(self):
		while not rospy.is_shutdown():
			GPSline = self.ser.readline(1)
			GPSline = codecs.decode(GPSline, "ascii")
			GPS_pars_data = nmea_parsing(GPSline, self.GPS_core)
			if GPS_pars_data != None:
				result = coord_transform(GPS_pars_data)
				if result != None:
					kalman_tm = self.kalman_df(result[0])
					heading = result[2]
					velocity = result[3]
					self.gps_cmd.tm_x = float(result[0][0])
					self.gps_cmd.tm_y = float(result[0][1])
					if (type(heading) == float):
						self.gps_cmd.yaw = float(heading)
					else:
						self.gps_cmd.yaw = float(0)
					#print("gps information = ",self.gps_cmd)
					self.publish()
					###kalman filter ###		
					'''					
					if kalman_tm is not None:
						print("heading:", heading, " velocity:", velocity)
						#test_ = result[0][0],result[0][1],kalman_tm[0], kalman_tm[1], heading, velocity
						print(result[0],kalman_tm)					
						#write.writerow(test_)
						#self.gps_cmd.tm_x = float(kalman_tm[0])
						#self.gps_cmd.tm_y = float(kalman_tm[1])
						if (type(heading) == float):
							self.gps_cmd.yaw = float(heading)
						else:
							self.gps_cmd.yaw = float(0)	
						#gps.publish()
					'''
	def publish(self):
		self.gps_pub.publish(self.gps_cmd)

if __name__ == '__main__':
	rospy.init_node('GPS',anonymous=True)
	gps = Gps()
	gps.parser()
