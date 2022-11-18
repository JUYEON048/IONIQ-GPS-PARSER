#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import matplotlib.pyplot as plt
# import openpyxl as op
# import numpy as np

import serial
import codecs
from GPS_preprocessing import coord_transform, nmea_parsing2
import rospy
import time
import numpy as np
from std_msgs.msg import String, Float32
from mi_msgs.msg  import GPS
import csv    
from datetime import datetime
from pykalman import KalmanFilter
from scipy import linalg
now = datetime.now()
parser = []
port = '/dev/ttyACM1'
baud = 115200
i = 0
check_kalman = []
def kalman_df(tm):
	global kf
	global x_now
	global P_now
	if len(check_kalman) == 0:
		check_kalman.append(tm)
	elif len(check_kalman) >= 1 and len(check_kalman) <2 :
		check_kalman.append(tm)
		measurements = np.asarray(check_kalman)
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
	elif len(check_kalman) >= 2:
		(x_now, P_now) = kf.filter_update(filtered_state_mean = x_now, filtered_state_covariance = P_now, observation = tm)
		kalman_tm = [x_now[0], x_now[2]]
		return kalman_tm
	


def publish_GPS(port, baud):
	ser = serial.Serial(port, baud, timeout=0)
	rospy.init_node('GPS_parser')
	#pub_TM = rospy.Publisher('gps/TM', String, queue_size=10)
	#pub_WGS = rospy.Publisher('gps/WGS', String, queue_size=10)	
	#pub_yaw = rospy.Publisher('gps/yaw', String, queue_size=10)
	gps_pub = rospy.Publisher('gps_messages',GPS,queue_size=10)
	GPS_core = []
	
	
	csv_file = open('TM_result-%s.csv'%(now.minute), 'w', encoding='utf-8', newline='')
	write = csv.writer(csv_file)
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		GPSline = ser.readline().strip()
		if len(GPSline) is not 0:
			GPSline = GPSline.decode('ascii','replace')
			GPS_pars_data = nmea_parsing2(GPSline, GPS_core)
			#print(GPS_pars_data)
			#if GPS_pars_data != None:
			result = coord_transform(GPS_pars_data)
			if result != None:
				
				kalman_tm = kalman_df(result[0])
				rate.sleep()
				heading = result[2]
				velocity = result[3]
				#print(type(result[0][0]), type(kalman_tm), type(heading), type(velocity))
				print(kalman_tm)
				if kalman_tm is not None:
					#print("heading:", heading, " velocity:", velocity)
					test_ = result[0][0],result[0][1],kalman_tm[0], kalman_tm[1], heading, velocity
					#print(result[0],kalman_tm)					
					write.writerow(test_)
					gps_cmd = GPS()
					gps_cmd.tm_x = float(kalman_tm[0])
					gps_cmd.tm_y = float(kalman_tm[1])
					if (type(heading) == float):
						gps_cmd.yaw = float(heading)
					else:
						gps_cmd.yaw = float(0)		
					gps_pub.publish(gps_cmd)
					#pub_TM.publish()
					#pub_yaw.publish(heading)
					#pub_WGS.publish(result[1])
				
	#csv_file.close()
		
if __name__ == '__main__':
	try:
		publish_GPS(port, baud)
	except rospy.ROSInterruptException:
		pass
