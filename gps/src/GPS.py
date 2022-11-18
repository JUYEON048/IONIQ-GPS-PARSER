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

    def __init__(self, writeable=True):
        self.port = '/dev/ttyACM0'
        self.baud = 115200
        self.gps_msg = GPS()
        self.ser = serial.Serial(self.port, self.baud, timeout=0)
        self.gps_core = []
        #self.timer = rospy.Timer(rospy.Duration(1.0/1000.0), self.parsing_data)
        self.gps_pub = rospy.Publisher('gps_messages', GPS, queue_size=1)
        self.csv_file = open('TM_result-%s.csv' % (now.minute), 'w', encoding='utf-8', newline='')
        self.write = csv.writer(self.csv_file)
        self.writeable = writeable
        self.kalman_idx = []
        self.kalman_tm = [0., 0.]
        self.heading = 0.
        self.velocity = 0.
        self.gps_time = 0.
        self.tm_xy = [[0., 0.], (0., 0.), self.heading, self.velocity, self.gps_time]
        self.tm = [0., 0.]
        self.bf_heading = -1000.0
        #self.kf_timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_coordinate)


    def parsing_data(self, event=None):
        self.rawdata = self.ser.readline().strip()
        #print(self.rawdata)
        if len(self.rawdata) is not 0:
            self.rawdata = self.rawdata.decode('ascii', 'replace')
            self.pdata = self.rawdata.split(',')
            if 'GNRMC' in self.pdata[0]:
                #print(self.pdata[1])
            
                self.tm_xy = coord_transform(self.pdata)
                
    def kalman_update(self):
        if len(self.kalman_idx) == 0:
            self.kalman_idx.append(self.tm_xy[0])
        elif len(self.kalman_idx) == 1:
            self.kalman_idx.append(self.tm_xy[0])
            self.measure = np.asarray(self.kalman_idx, dtype=np.float16)
            self.init_state_mean = [self.measure[0, 0], 0, self.measure[0, 1], 0]
            self.trans_mat = [[1, 1, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 1],
                              [0, 0, 0, 1]]
            self.observ_mat = [[1, 0, 0, 0],
                               [0, 0, 1, 0]]
            kf = KalmanFilter(transition_matrices=self.trans_mat,
                                   observation_matrices=self.observ_mat,
                                   initial_state_mean=self.init_state_mean,
                                   em_vars=['transition_covariance', 'initial_state_covariance'])

            self.kf = kf.em(self.measure)
            (bar_state_mean, bar_state_cov) = self.kf.filter(self.measure)
            self.x_ = bar_state_mean[-1, :]
            self.p_ = bar_state_cov[-1, :]
        else:
            (self.x_, self.p_) = self.kf.filter_update(filtered_state_mean=self.x_,
                                                       filtered_state_covariance=self.p_,
                                                       observation=self.tm_xy[0])
            self.kalman_tm = [self.x_[0], self.x_[2]]
            
        return self.kalman_tm


    def publish_coordinate(self,event=None):
        if self.tm_xy is not None:
            self.kalman_tm = self.kalman_update()
            self.tm = self.tm_xy[0]
            self.wgs = self.tm_xy[1]
            self.heading = self.tm_xy[2]
            self.velocity = self.tm_xy[3]
            self.gps_time = self.tm_xy[4]

            self.gps_msg.velocity = self.velocity
            self.gps_msg.kf_x = float(self.kalman_tm[0])
            self.gps_msg.kf_y = float(self.kalman_tm[1])
            self.gps_msg.gps_time = float(self.gps_time)
            self.gps_msg.tm_x = float(self.tm[0])
            self.gps_msg.tm_y = float(self.tm[1])
            self.gps_msg.lati = float(self.wgs[0])
            self.gps_msg.long = float(self.wgs[1])
            self.gps_msg.header.stamp = rospy.Time.now() 
            if type(self.heading) == float:
                self.gps_msg.heading = float(self.heading)
                self.bf_heading = float(self.heading)
            elif self.heading == 0.0:
                self.gps_msg.heading = self.bf_heading
            else:
                self.gps_msg.heading = -1000.0
            self.gps_pub.publish(self.gps_msg)
            print(self.gps_msg)
            if self.writeable is True:
                write_info = self.tm_xy[0][0], self.tm_xy[0][1], self.kalman_tm[0], self.kalman_tm[1], self.heading, self.velocity
                self.write.writerow(write_info)

# csv_file.close()

if __name__ == '__main__':
    rospy.init_node('GPS_parser')
    GPS = gps()
    rospy.Timer(rospy.Duration(1.0/50.0), GPS.parsing_data)
    rospy.Timer(rospy.Duration(1.0 /50.0), GPS.publish_coordinate)
    rospy.spin()












































































































