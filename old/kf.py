import math #used for atan
import time

class kalmanFilter:

    #initialize
	def __init__(self):
		self.Q_angle = float(0.01) #this is a variance
		self.Q_gyro = float(0.0003) #this is a variance
		self.R_angle = float(0.01) #this is a variance
		self.x_bias = float(0)
		self.y_bias = float(0)
		self.z_bias = float(0)
		self.L = float(100) #this is just a random large number used in the starting calibration
		self.XP_00, self.XP_01, self.XP_10, self.XP_11 = float(self.L), float(0), float(0), float(self.L) #L can be 0 but should be a high number
		self.YP_00, self.YP_01, self.YP_10, self.YP_11 = float(self.L), float(0), float(0), float(self.L)
		self.ZP_00, self.ZP_01, self.ZP_10, self.ZP_11 = float(self.L), float(0), float(0), float(self.L)
		self.KFangleX = float(0.0)
		self.KFangleY = float(0.0)
		self.KFangleZ = float(0.0)

    #Y filter call
	#accAngle = angle measured with atan2 using the accelerometer
	#gyroRate = angle measured using the gyroRate
	#looptime = loop time in millis()
	def Y(self, accAngle, gyroRate, looptime):
	
		#turns milliseconds into seconds
		DT = float(float(looptime)/1000)
	
		##predict
		#this predicts the current state (()calculates rate of angle change - bias) * time passed))
		self.KFangleY += + DT * (gyroRate - self.y_bias)

		#this is estimating the error covariance and puts it into a matrix, the important ones are 00 and 11
		self.YP_00 += - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT
		self.YP_01 += - DT * self.YP_11
		self.YP_10 += - DT * self.YP_11
		self.YP_11 += + self.Q_gyro * DT

		##update
		#finds the innovation (measured angle - estimated angle)
		y = float(accAngle - self.KFangleY)
		#calculate the innovation covariance (uses updated error covariance matrix and noise bias) (this is basically how much we trust the measurements)
		S = float(self.YP_00 + self.R_angle)
		#calculate kalman gain using innovation covariance
		K_0 = float(self.YP_00 / S)
		K_1 = float(self.YP_10 / S)

		##final
		#give estimated current state based on innovation and kalman gain
		self.KFangleY += K_0 * y
		#update the bias
		self.y_bias += K_1 * y
		
		#update the error covariance matrix using the kalman gain
		YP00_temp = float(self.YP_00) #temporary
		YP01_temp = float(self.YP_01) #temporary
		
		self.YP_00 -= K_0 * YP00_temp
		self.YP_01 -= K_0 * YP01_temp
		self.YP_10 -= K_1 * YP00_temp
		self.YP_11 -= K_1 * YP01_temp

		#return the estimated current angle
		return self.KFangleY

    #X filter call
	def X(self, accAngle, gyroRate, looptime):
	
		DT = float(float(looptime)/1000)	
	
		self.KFangleX += + DT * (gyroRate - self.x_bias)
		self.XP_00 += - DT * (self.XP_10 + self.XP_01) + self.Q_angle * DT
		self.XP_01 += - DT * self.XP_11
		self.XP_10 += - DT * self.XP_11
		self.XP_11 += + self.Q_gyro * DT

		y = float(accAngle - self.KFangleX)
		S = float(self.XP_00 + self.R_angle)
		K_0 = float(self.XP_00 / S)
		K_1 = float(self.XP_10 / S)

		self.KFangleX += K_0 * y
		self.x_bias += K_1 * y
		
		#update the error covariance matrix using the kalman gain
		XP00_temp = float(self.XP_00) #temporary
		XP01_temp = float(self.XP_01) #temporary
		
		self.XP_00 -= K_0 * XP00_temp
		self.XP_01 -= K_0 * XP01_temp
		self.XP_10 -= K_1 * XP00_temp
		self.XP_11 -= K_1 * XP01_temp

		return self.KFangleX

	#for z
	def Z(self, accAngle, gyroRate, looptime):
	
		#turns milliseconds into seconds
		DT = float(float(looptime)/1000)
	
		##predict
		#this predicts the current state (()calculates rate of angle change - bias) * time passed))
		self.KFangleZ += + DT * (gyroRate - self.z_bias)

		#this is estimating the error covariance and puts it into a matrix, the important ones are 00 and 11
		self.ZP_00 += - DT * (self.ZP_10 + self.ZP_01) + self.Q_angle * DT
		self.ZP_01 += - DT * self.ZP_11
		self.ZP_10 += - DT * self.ZP_11
		self.ZP_11 += + self.Q_gyro * DT

		##update
		#finds the innovation (measured angle - estimated angle)
		y = float(accAngle - self.KFangleZ)
		#calculate the innovation covariance (uses updated error covariance matrix and noise bias) (this is basically how much we trust the measurements)
		S = float(self.ZP_00 + self.R_angle)
		#calculate kalman gain using innovation covariance
		K_0 = float(self.ZP_00 / S)
		K_1 = float(self.ZP_10 / S)

		##final
		#give estimated current state based on innovation and kalman gain
		self.KFangleZ += K_0 * y
		#update the bias
		self.z_bias += K_1 * y
		
		#update the error covariance matrix using the kalman gain
		ZP00_temp = float(self.ZP_00) #temporary
		ZP01_temp = float(self.ZP_01) #temporary
		
		self.ZP_00 -= K_0 * ZP00_temp
		self.ZP_01 -= K_0 * ZP01_temp
		self.ZP_10 -= K_1 * ZP00_temp
		self.ZP_11 -= K_1 * ZP01_temp

		#return the estimated current angle
		return self.KFangleZ

def AAX(accx, accy, accz):
	return math.degrees(math.atan2(accy,accz))#i know this one is right

def AAZ(accx, accy, accz):
	return math.degrees(math.atan2(accx,accy)) #x and y might have to be switched

def AAY(accx, accy, accz):
	return math.degrees(math.atan2(accz,accx)) #z and x might have to be switched


