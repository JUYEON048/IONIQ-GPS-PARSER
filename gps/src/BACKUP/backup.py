#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import codecs
from GPS_preprocessing import coord_transform, nmea_parsing2
import rospy
import time
import numpy as np
from std_msgs.msg import String, Float32
from mi_msgs.msg import GPS
import csv
from datetime import datetime
from pykalman import KalmanFilter
from scipy import linalg


now = datetime.now()

class gps():

    def __init__(self):
        self.port = '/dev/ttyACM0'
        self.baud = 115200
        self.ser = serial.Serial(self.port, self.baud, timeout=0)
        self.gps_core = []
        self.result = []
        self.timer = rospy.Timer(rospy.Duration(1.0/1000.0), self.parsing_data)
    def parsing_data(self, timer):
        self.rawdata = self.ser.readline().strip()
        if len(self.rawdata) is not 0:
            self.rawdata = self.rawdata.decode('ascii', 'replace')
            self.parsing_data = self.rawdata.split(',')
            if 'GNRMC' in self.parsing_data[0]:
                self.result = coord_transform(self.parsing_data)
                #print(self.result)
class kalman(gps):

    def __init(self, writeable=True):
        self.gps_pub = rospy.Publisher('gps_messages', GPS, queue_size=1)
        self.csv_file = open('TM_result-%s.csv' % (now.minute), 'w', encoding='utf-8', newline='')
        self.write = csv.writer(self.csv_file)
        self.writeable = writeable
        self.kalman_idx = []
        self.tm_xy = super(kalman, self).parsing_data()
        print(self.tm_xy)
        self.init_state_mean = [self.measure[0, 0], 0, self.measure[0, 1], 0]
        self.trans_mat = [[1, 1, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 1],
                         [0, 0, 0, 1]]
        self.observ_mat = [[1, 0, 0, 0],
                           [0, 0, 1, 0]]
        self.kf = KalmanFilter(transition_matrices=self.trans_mat,
                          observation_matrices=self.observ_mat,
                          initial_state_mean=self.init_state_mean,
                          em_vars=['transition_covariance', 'initial_state_covariance'])
        self.kalman_tm = [0., 0.]
        self.heading = 0.
        self.velocity = 0.
        self.kf_timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_coordinate)

    def kalman_update(self):
        if len(self.kalman_idx) == 0:
            self.kalman_idx.append(self.tm_xy)
        elif len(self.kalman_idx) == 1:
            self.kalman_idx.append(self.tm_xy)
            self.measure = np.asarray(self.kalman_idx, dtype=np.float16)
            self.kf = self.kf.em(self.measure)
            (bar_state_mean, bar_state_cov) = self.kf.filter(self.measure)
            self.x_ = bar_state_mean[-1, :]
            self.p_ = bar_state_cov[-1, :]
        else:
            (self.x_, self.p_) = self.kf.filter_update(filtered_state_mean=self.x_,
                                                       filtered_state_covariance=self.p_,
                                                       observation=self.tm_xy)
            self.kalman_tm = [self.x_[0], self.x_[2]]
        return self.kalman_tm


    def publish_coordinate(self,kf_timer):
        print(self.tm_xy)
        if self.tm_xy is not None:
            self.kalman_tm = self.kalman_update(self.tm_xy[0])
            print(self.kalman_tm)

            self.heading = self.tm_xy[2]
            self.velocity = self.tm_xy[3]
            gps = GPS()
            gps.tm_x = float(self.kalman_tm[0])
            gps.tm_y = float(self.kalman_tm[1])

            if type(self.heading) == float:
                gps.yaw = float(self.heading)
            else:
                gps.yaw = float(0.)

            self.gps_pub.publish(gps)

            if self.writeable is True:
                write_info = self.tm_xy[0][0], self.tm_xy[0][1], self.kalman_tm[0], self.kalman_tm[1], self.heading, self.velocity
                self.write.writerow(write_info)

# csv_file.close()

if __name__ == '__main__':
    rospy.init_node('GPS_parser')
    #GPS = gps()
    KF = kalman()

    rospy.spin()












































































































