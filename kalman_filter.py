import rospy
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Int32MultiArray

class KalmanFilter(object):
	'''
	Kalman filtresinin başlangıç durumu ve parametreleri 
	Ayrıca ROS düğümünü başlatır ve GPS ve IMU verilerini dinlemek için 
	abonelikler oluşturur.
	'''
	def __init__(self):
		
			rospy.init_node("kalman_filter", anonymous = True)
			self.gps = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_data_callback)
			self.imu = rospy.Subscriber('/imu', Imu, self.imu_data_callback)
			self.pub = rospy.Publisher('topic2', Int32MultiArray, queue_size = 5)


			self.origin_latitude = 40.9955106085895
			self.origin_longitude = 29.062988181734983
			self.x_coordinate = 0
			self.y_coordinate = 0
			self.robot_latitude = 0
			self.robot_longitude = 0
			self.gps_covariance = 0
			self.x_acceleration = 0
			self.y_acceleration = 0
			self.dt = 0.5 
			
			
			self.xk = np.matrix([[0],[0],[0],[0]])
			
			self.F = np.matrix([[1, 0, self.dt, 0],
								[0, 1, 0, self.dt],
								[0, 0, 1, 0],
								[0, 0, 0, 1]])
								
			self.G = np.matrix([[0.5*self.dt**2, 0,],
								[0, 0.5*self.dt**2,],
								[self.dt, 0],
								[0, self.dt]])
								
								
			self.H = np.matrix([[1,0,0,0],
								[0,1,0,0]])
								
			
			self.Q=np.matrix([	[0.001,0,0,0],
					 			[0,0.001,0,0],
								[0,0,0.001,0],
								[0,0,0,0.001]])
			
			self.R = np.matrix([[0.002,0],
								[0,0.002]])
								
			self.P = np.matrix([[0.002,0,0,0],
					   			[0,0.002,0,self.dt],
								[0,0,0,0],
								[0,0,0,0]])
			
	'''
	GPS sensörden gelen verileri işler.
	'''
	def gps_data_callback(self,msg):
			self.robot_latitude = msg.latitude
			self.robot_longitude = msg.longitude
			self.gps_covariance = msg.position_covariance
			
	'''
	IMU sensörden gelen verileri işler.
	'''		
	def imu_data_callback(self,msg):
			self.x_acceleration = msg.linear_acceleration.x
			self.y_acceleration = msg.linear_acceleration.y
			self.acceleration_convariance = msg.linear_acceleration_covariance
			
	'''
	GPS koordinatlarını yerel koordinatlara dönüştürür.
	'''
	def convert_local_coordinates(self,lat,lon):
			
			WORLD_POLAR_M = 6356752.3142
			WORLD_EQUATORIAL_M = 6378137.0
			
			eccentricity = math.acos(WORLD_POLAR_M / WORLD_EQUATORIAL_M)
			n_prime = 1 / (math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))), 2.0) * math.pow(math.sin(eccentricity), 2.0)))
			m = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)
			n = WORLD_EQUATORIAL_M * n_prime
			
			diffLon = float(lon) - float(self.origin_longitude)
			diffLat = float(lat) - float(self.origin_latitude)
			
			pi_over_180 = math.pi / 180.0
			surfdistLon = pi_over_180 * math.cos(math.radians(float(lat))) * n
			surfdistLat = pi_over_180 * m
			
			x = diffLon * surfdistLon
			y = diffLat * surfdistLat
			
			return x, y
			
	'''
	Kalman filtresinin tahmin adımlarını gerçekleştirir.
	'''	
	def prediction(self):		
			
			accel = np.matrix([[self.x_acceleration],[self.y_acceleration]])
			
			x_k = np.dot(self.F, self.xk) + np.dot(self.G, accel)
			p_k = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
			
			return x_k,p_k
	
	'''
	Kalman filtresinin güncelleme adımlarını gerçekleştirir.
	'''		
	def correction(self,x_k,p_k,Y):
			
			S = np.dot(self.H, np.dot(p_k, self.H.T)) + self.R
			K = np.dot(np.dot(p_k, self.H.T), np.linalg.inv(S))
			self.xk = np.round(x_k + np.dot(K, (Y - np.dot(self.H, x_k))))
			I = np.eye(self.H.shape[1])
			self.P = (I - (K * self.H)) * p_k
			pos_x=float(self.xk[0])
			pos_y=float(self.xk[1])
			
			return pos_x,pos_y
			
	'''
	Sürekli olarak Kalman filtresi çalıştırılır, 
	tahmin edilen konumu yazdırır ve ROS üzerinden yayınlar
	'''		
	def run_filter(self):
			while(True):
				self.x_coordinate, self.y_coordinate = self.convert_local_coordinates(self.robot_latitude, self.robot_longitude)
				Y = np.array([[self.x_coordinate], [self.y_coordinate]])
				x_k, p_k = self.prediction()
				(x1, y1) = self.correction(x_k, p_k, Y)
				rospy.sleep(0.4)
				a = Int32MultiArray()
				a.data = []
				a.data.append(float(x1))
				a.data.append(float(y1))
				self.pub.publish(a)
				print(f"x: {x1:.2f}, y: {y1:.2f}")
					
						
Robot_Kalman = KalmanFilter()
Robot_Kalman.run_filter()
rospy.spin()
