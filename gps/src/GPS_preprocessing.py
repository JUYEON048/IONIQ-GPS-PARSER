import pyproj
import numpy as np
def nmea_parsing(data2, GPS_core):
	if data2 == '':
		pass

	if data2 == "$":
		GPS_ = ''.join(GPS_core)
		parser = GPS_.split(',')
		del GPS_core[:]

	else:
		GPS_core.append(data2)
		parser = None
	return parser
def nmea_parsing2(data2, GPS_core):
	parser = data2.split(',')
	return parser
def WGS_to_TM(Degree_longtitude, Degree_latitude):
	wgs84 = [Degree_latitude, Degree_longtitude]
	LATLONG_WGS84 = pyproj.Proj("+proj=latlong +datum=WGS84 +ellps=WGS84")
	TM127= pyproj.Proj("+proj=tmerc +lat_0=38N +lon_0=127E +ellps=bessel +x_0=200000 +y_0=600000 +k=1.0 ")
	longt = Degree_longtitude
	lat = Degree_latitude
	x,y = pyproj.transform(LATLONG_WGS84, TM127, longt, lat)
	TM = [x,y]	
	return TM

def to_degree(raw_longtitude, raw_latitude):
	deg_latitude = (int)(raw_latitude)
	deg_longtitude = (int)(raw_longtitude)

	min_latitude = (raw_latitude - deg_latitude) * 100
	min_longtitude = (raw_longtitude - deg_longtitude) * 100
	
	dmin_latitude = (int)((raw_latitude - deg_latitude) * 60)
	dmsec_latitude = (float)((((raw_latitude - deg_latitude) * 60) - dmin_latitude) * 60)
	
	dmin_longtitude = (int)((raw_longtitude - deg_longtitude) * 60)
	dmsec_longtitude = (float)((((raw_longtitude - deg_longtitude) * 60) - dmin_longtitude) * 60)

	Degree_latitude = deg_latitude + (min_latitude / 60)
	Degree_longtitude = deg_longtitude + (min_longtitude / 60)

	return Degree_longtitude, Degree_latitude


"""
for measurement in measurements:
    #time_before = time.time()
    (x_now, P_now) = kf3.filter_update(filtered_state_mean = x_now,
                                       filtered_state_covariance = P_now,
                                       observation = measurement)
    data_ = [x_now[0], x_now[2]]
	
def coord_transform(data):

	if data[0] == 'GNGGA':
		raw_latitude = (float)(data[2]) * 0.01
		raw_longtitude = (float)(data[4]) * 0.01
		hight = (float)(data[9])

		Degree_longtitude, Degree_latitude = to_degree(raw_longtitude, raw_latitude)
		WGS = Degree_longtitude, Degree_latitude		
		TM = WGS_to_TM(Degree_longtitude, Degree_latitude)
		result = TM, WGS
		return result
	else:
		pass
"""

def coord_transform(data):
	"""
	if 'GNRMC' in data[0]:# and data[3] != '':
		#print("data = ",data)
		raw_latitude = (float)(data[3]) * 0.01
		#print("raw_lat",raw_latitude)
		raw_longtitude = (float)(data[5]) * 0.01
		velocity = (float)(data[7]) * 1.852
		Degree_longtitude, Degree_latitude = to_degree(raw_longtitude, raw_latitude)
		WGS = Degree_longtitude, Degree_latitude		
		TM = WGS_to_TM(Degree_longtitude, Degree_latitude)
	


		try:
			True_heading = (float)(data[8])
		except ValueError:
			True_heading = 0.0
		result = TM, WGS, True_heading, velocity
		return result
	else:
		pass

	"""
	raw_latitude = (float)(data[3]) * 0.01
	# print("raw_lat",raw_latitude)
	raw_longtitude = (float)(data[5]) * 0.01
	velocity = (float)(data[7]) * 1.852
	Degree_longtitude, Degree_latitude = to_degree(raw_longtitude, raw_latitude)
	WGS = Degree_longtitude, Degree_latitude
	TM = WGS_to_TM(Degree_longtitude, Degree_latitude)
	time = (float)(data[1])
	
	try:
		
		True_heading = (float)(data[8])
		
	except ValueError:
		True_heading = 0
		
	result = TM, WGS, True_heading, velocity, time
	return result
