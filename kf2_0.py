import math #used for atan
import time #used for delta_t
from datetime import datetime
import matplotlib.pyplot as plt #used for graphing
import random #for creating noise
import accxbee
import curses
import thread
import xbeelib
import sys
import numpy
footstep_multiplier = 2.5 #indicates the length of vector multiply by this number, used for detecting the footstep 

class kalmanFilter:

    #initialize
	def __init__(self):
		self.Q_angle = float(0.01) #this is a variance
		self.Q_gyro = float(0.0003) #this is a variance
		self.R_angle = float(0.01) #this is a variance
		self.x_bias = float(0)
		self.y_bias = float(0)
		self.z_bias = float(0)
		self.L = float(0) #this is just a random large number used in the starting calibration
		self.XP_00, self.XP_01, self.XP_10, self.XP_11 = float(self.L), float(0), float(0), float(self.L) #L can be 0 but should be a high number
		self.YP_00, self.YP_01, self.YP_10, self.YP_11 = float(self.L), float(0), float(0), float(self.L)
		self.ZP_00, self.ZP_01, self.ZP_10, self.ZP_11 = float(self.L), float(0), float(0), float(self.L)
                self.gyo_min = [0,0,0]
                self.gyo_max = [0,0,0]
                self.gyo_mean = [0,0,0]
		self.KFangleX = float(0.0)
		self.KFangleY = float(0.0)
		self.KFangleZ = float(0.0)
		self.pitch = float(0)
		self.roll = float(0)
		self.gmag = float(0)
		self.g = [0,0,0] #holds xyz of grav
		self.timer_mark = float(int(round(time.time()*1000))) #get time at initialization in milliseconds (used only for graphs)
		self.imulib = accxbee.IMU() #initialize the accxbee script and the imu class, for getting value from the IMU module
		self.xbeelib = xbeelib.XBEE(self.imulib) #initialize the xbeelib script and the xbee class, for the XBEE transmit and receive
		#xbeelib.setIMU(imulib)
		self.magangle = 0
		self.delta_t = 0.02 

        def calibrateGyro(self, gyro):
                if gyro[0]<self.gyo_max[0] and gyro[0] > self.gyo_min[0]:
                        gyro[0] = 0
                else:
                        gyro[0] = gyro[0] - self.gyo_mean[2]
                if gyro[1]<self.gyo_max[1] and gyro[1] > self.gyo_min[1]:
                        gyro[1] = 0
                else:
                        gyro[0] = gyro[0] - self.gyo_mean[2]
                if gyro[2]<self.gyo_max[2] and gyro[2] > self.gyo_min[2]:
                        gyro[2] = 0
                else:
                        gyro[2] = gyro[2] - self.gyo_mean[2]
                return gyro


	def setgmag(self, grav):
		self.gmag = grav		
	
	def setMagAngle(self, angle):
		self.magangle = angle
	def setPitch(self, angle):
		self.pitch = angle
	def setRoll(self, angle):
		self.roll = angle

	def updateG(self):
		#sets the g array
		self.g[0] = math.cos(self.KFangleX) * self.gmag
		self.g[1] = math.cos(self.KFangleY) * self.gmag
		self.g[2] = math.cos(self.KFangleZ) * self.gmag
	
	def getgmag(self):
		return self.gmag
		
	def getTimer_Mark(self):
		return self.timer_mark
		
	def getG(self):
		return self.g

	def calcDistance(self, d, o, a, v, t):
		#take acceleration and orientation
		
		#update velocity
		v[0] = v[0]+(a[0]*t)/1000
		v[1] = v[1]+(a[1]*t)/1000
		v[2] = v[2]+(a[2]*t)/1000
		
		#update distance
		d[0] = d[0] + (v[0]*t)/1000 + ((a[0]*t*t)/2)/(1000*1000)
		d[1] = d[1] + (v[1]*t)/1000 + ((a[1]*t*t)/2)/(1000*1000)
		d[2] = d[2] + (v[2]*t)/1000 + ((a[2]*t*t)/2)/(1000*1000)
		
		return  d, v
	
    #Y filter call
	#accAngle = angle measured with atan2 using the accelerometer
	#gyroRate = angle measured using the gyroRate
	#looptime = loop time in millis()
	def Y(self, accAngle, gyroRate, looptime):
		#correct theangle
		#accAngle = anglecorrection(accAngle)
	
		#turns milliseconds into seconds
		#DT = float(float(looptime)/1000)
		DT = looptime
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
		#correct the angle
		#accAngle = anglecorrection(accAngle)
	
		#DT = float(float(looptime)/1000)	
		DT = looptime
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
		#correct theangle
		#accAngle = anglecorrection(accAngle)
		
		#turns milliseconds into seconds
		#DT = float(float(looptime)/1000)
		DT = looptime

		##predict
		#this predicts the current state (()calculates rate of angle change - bias) * time passed))
		self.KFangleZ +=  DT * (gyroRate - self.z_bias)

		#this is estimating the error covariance and puts it into a matrix, the important ones are 00 and 11
		#self.ZP_00 += - DT * (self.ZP_10 + self.ZP_01) + self.Q_angle * DT
		#self.ZP_01 += - DT * self.ZP_11
		#self.ZP_10 += - DT * self.ZP_11
		#self.ZP_11 += + self.Q_gyro * DT

		##update
		#finds the innovation (measured angle - estimated angle)
		#y = float(accAngle - self.KFangleZ)
		#calculate the innovation covariance (uses updated error covariance matrix and noise bias) (this is basically how much we trust the measurements)
		#S = float(self.ZP_00 + self.R_angle)
		#calculate kalman gain using innovation covariance
		#K_0 = float(self.ZP_00 / S)
		#K_1 = float(self.ZP_10 / S)

		##final
		#give estimated current state based on innovation and kalman gain
		#self.KFangleZ += K_0 * y
		#update the bias
		#self.z_bias += K_1 * y
		
		#update the error covariance matrix using the kalman gain
		#ZP00_temp = float(self.ZP_00) #temporary
		#ZP01_temp = float(self.ZP_01) #temporary
		
		#self.ZP_00 -= K_0 * ZP00_temp
		#self.ZP_01 -= K_0 * ZP01_temp
		#self.ZP_10 -= K_1 * ZP00_temp
		#self.ZP_11 -= K_1 * ZP01_temp

		#return the estimated current angle
		return self.KFangleZ
		


def calibrate_filter(kf):	#runs a 15 second calibration loop on the filter
	print("Begin calibration routine")
	
	#start graph
	
	#plt.suptitle("calibration routine")
	#graph_init()
	
	#setup timers
	timer_mark = float(kf.getTimer_Mark()) #this is time since start of program
	timer_mark_loop = float(round(time.time()*1000))
	time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
	time_since_start = float(round(time.time()*1000) - timer_mark)
	
	acc = [0,0,0] #this is an average
	orientation = [0,0,0]
	magangle = 0 #angle calculated by using magnetometer
	count = 0
	#set the initial value with the reading from magnetometer
	kf.KFangleZ = math.radians( mag_compass(kf.imulib.get_mag_x() , kf.imulib.get_mag_y()) )
	#time.sleep(0.05)
	
	buffx = []
        buffy = []
        buffz = []
        #take 50 reading from the gyroscope first
        for tmp in range(50):
                gyro = kf.imulib.get_gyo_all()
                for_loop_timer = float( round(time.time()*1000) )
                buffx.append(gyro[0])
                buffy.append(gyro[1])
                buffz.append(gyro[2])
                time.sleep(0.002)

        kf.gyo_min[0] = min(buffx)
        kf.gyo_min[1] = min(buffy)
        kf.gyo_min[2] = min(buffz)

        kf.gyo_max[0] = max(buffx)
        kf.gyo_mean[0] = numpy.mean(buffx)
        kf.gyo_max[1] = max(buffy)
        kf.gyo_mean[1] = numpy.mean(buffy)
        kf.gyo_max[2] = max(buffz)
        kf.gyo_mean[2] = numpy.mean(buffz)



	while (time_since_start < 20000): #stop after 7 seconds
		#calculate time since start
		time_since_start = float(round(time.time()*1000) - timer_mark )
		#time since loop start
		time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
		#calculate delta_t
		timer = float(round(time.time())) #get time before loop in milliseconds
		
		#use kf.imulib to access the accxbee.py for getting the imu data
		#kf is KalmanFilter class
		accel = kf.imulib.get_acc_all()
		gyro = kf.imulib.get_gyo_all()
                gyro = kf.calibrateGyro(gyro)
                accx = accel[0]
                accy = accel[1]
                accz = accel[2]
                gyox = gyro[0]
                gyoy = gyro[1]
                gyoz = gyro[2]

		#get angles
		#update sensors()
		#calculate acceleration angles
		aax = float(AAX(accx,accy,accz)) #x
		aaz = float(AAZ(accx,accy,accz)) #z
		aay = float(AAY(accx,accy,accz)) #y
		
		#just a holder for the mag angle

		#calculating alpha which is for averaging in the acceleration
		#while (float(round(time.time()*1000))- time_since_loop_start < 20):
			#print "sleeping to meet 20ms DT"
		#	time.sleep(.001) #we need a sleep here because otherwise the program runs too fast
		#print float(round(time.time()*1000))- time_since_loop_start
		
		#delta_t = float(round(time.time()*1000) - timer) #calculates delta time since start of a loop

		#sum of all accelerations
		acc[0] = acc[0]+accx
		acc[1] = acc[1]+accy
		acc[2] = acc[2]+accz

		magangle =  magangle + mag_compass(kf.imulib.get_mag_x() , kf.imulib.get_mag_y())

		count+=1
		
		#print("accelerometer averages", acc)
		
		#print("accelerometer angle: x-",aax," y-",aay," z-",aaz) #for some reason if you comment out this code, everything dies

		#delta_t = int(round(time.time()*1000)) - timer #reset dt each calculation
		delta_t = kf.delta_t
		#print ('delta_t:',delta_t)
		KangleX = kf.X(aax, gyox, delta_t) #gets calculated x
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleY = kf.Y(aay, gyoy, delta_t) #gets calculated y
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleZ = kf.Z(math.radians( mag_compass(kf.imulib.get_mag_x() , kf.imulib.get_mag_y())) , gyoz, delta_t)
		#TODO: need to change the way the filter calculate Z axis 
		#KangleZ = KangleZ + magangle
		#KangleZ = kf.Z(aaz, gyoz, delta_t) #gets calculated z
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		#orientation = [KangleX, KangleY, KangleZ] #doesnt need to be average because the filter already does it
		while float(round(time.time())) - timer < 0.02:
			time.sleep( 0.001 )
		#calibration graph
		#graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
	
	#calibration completed, orientation should be correct
	#important!!! setting gravity should only happen once
	#average accelerations from count
	acc[0]=acc[0]/count
	acc[1]=acc[1]/count
	acc[2]=acc[2]/count
	#print 'magangle / count: '+str(magangle/count)
	#kf.KFangleZ = math.radians(magangle / count)
	
	mag = calcMag(acc) #magnitude of gravity vector
	kf.setgmag(mag) #set the kalman filter grav
	kf.updateG() #updates the component vectors of g from the magnitude and angles
	'''
	print("orientation",orientation)
	print("acceleration",acc)
	print("acc-g",removeG(acc, kf.getG()))
	'''
	print 'orientation: '+str(math.degrees( kf.KFangleZ ))
	print("gravity magnitude", kf.getgmag())
	print("gravity removed", kf.getG())
	print("end of calibration")
	
	#close calibration graph and sleep for a bit
	
	#return the calibrated filter
	#print("sleeping")
	#time.sleep(1) #remove this later
	#print("clearing graph")
	#plt.clf() #clears figure
	return kf
	
	
		
	
	
def footstep_length(height): #height should be in meter
	return height * 0.4

#angle correction when it hits 180 (used in kf object)
def anglecorrection(tmp):
	if tmp <= 360 and tmp > 0:
                return tmp
        elif tmp < 0 and tmp > -360:
                return 360 + tmp
        elif tmp < -360:
                return math.fabs(tmp % -360)
        else:
                return tmp % 360
def calcMag(m):
	return float(math.sqrt(m[0]*m[0]+m[1]*m[1]+m[2]*m[2]))

#acceleration angle X
def AAX(accx, accy, accz): 
	return math.acos(accx/calcMag([accx,accy,accz])) 
#Acceleration angle Z
def AAZ(accx, accy, accz):
	return math.acos(accz/calcMag([accx,accy,accz]))
#Acceleration angle Y
def AAY(accx, accy, accz):
	return math.acos(accy/calcMag([accx,accy,accz]))

#need to verify
#rotate along the X axis 
def findPitch(accx, accy, accz):
	den =  math.sqrt(accx*accx+accz*accz)
	#print 'accx:'+str(accx) + '    math.sqrt(accy*accy+accz*accz):'+ str(den)
	if den > 0:
		return math.atan( accx / den)
	else:
		return 0

	#return math.atan( accx/ math.sqrt(accy*accy+accz*accz) )
#need to verify
#rotate along the Y axis
def findRoll(accx, accy, accz):
	den = math.sqrt(accx*accx+accz*accz) #separate the denominator out so that the app doesnt need to do complex calculation multiple time
	#print 'accy:'+str( accy) + '    math.sqrt(accx*accx+accz*accz):'+ str( den )
	if den > 0:
		return math.atan(accy / den)
	else:
		return 0
        #return math.atan( accy/ math.sqrt(accx*accx+accz*accz) )

def findHeading(roll, pitch, accx, accy, accz):
	heading = 0
	xheading = 0
	yheading = 0
	if roll==0 and pitch==0:
		if not accx==0:
			heading = math.atan( accy/accx )
		else:
			heading = 0
	else:
		xheading = accx * math.cos(pitch) + accy*math.sin(pitch)*math.sin(roll) + accz * math.sin(pitch)*math.cos(roll)
		yheading = accy * math.cos(roll) - accz*math.sin(roll)
		#print 'xheading:'+str(xheading) + '  yheading:'+str(yheading)
		heading = math.fabs (math.degrees( math.atan( -yheading / xheading ))) if not xheading==0 else 0
		#if xheading < 0 and yheading > 0: #negative heading angle, 2 quadrant
		#	heading = 180 - heading
                #elif xheading > 0 and yheading < 0:# negative heading angle, 4 quadrant
                #        heading = 360 - heading

                #elif xheading > 0 and yheading > 0: #positive heading angle, 1 quadrant
                #        heading = heading
		#elif xheading < 0 and yheading < 0: #positive heading angle, 3 quadrant
		#	heading = 270 - heading
                
		#elif xheading == 0 and yheading < 0:
                #        heading = 90

                #elif xheading == 0 and yheading > 0:
                #        heading = 270

	return heading #this should be degree


def rad_to_angle(input):
	tmp = math.degrees(input)
	#print 'input:'+str(input)+'   tmp:'+str(tmp)
	if tmp <= 360 and tmp > 0:
		return tmp
	elif tmp < 0 and tmp > -360:
		return 360 + tmp
	elif tmp < -360:
		return math.fabs(tmp % -360)
	else:
		return tmp % 360

#initialize the graph
def graph_init():
    #set up graph stuff
	plt.ion() #activates interactive plots
	plt.show() #shows the graph
	#plt.suptitle("Calibration routine")

	#labels
	#x
	plt.subplot(3,1,1)
	plt.xlabel("time (ms)")
	plt.ylabel("Angel (radians)")
	plt.title("x axis")
	
	#y
	plt.subplot(3,1,2)
	plt.xlabel("time (ms)")
	plt.ylabel("Angle (radians)")
	plt.title("y axis")
	
	#z
	plt.subplot(3,1,3)
	plt.xlabel("time (ms)")
	plt.ylabel("Angel (radians)")
	plt.title("z axis")
	
	plt.tight_layout() #fixes layout issues
	
	return

#updates the draw on graph
def graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz):
	#x
	plt.subplot(3,1,1)
	plt.scatter(time_since_start,KangleX, c=u'r') #print kx angle
	plt.scatter(time_since_start,gyox, c=u'b', marker=u'.')  #print gyro x
	plt.scatter(time_since_start,aax, c=u'g', marker=u'x')  #print acc x

	#y
	plt.subplot(3,1,2)
	plt.scatter(time_since_start,KangleY, c=u'r') #print ky angle
	plt.scatter(time_since_start,gyoy, c=u'b', marker=u'.')  #print gyro y
	plt.scatter(time_since_start,aay, c=u'g', marker=u'x')  #print acc y

	#z
	plt.subplot(3,1,3)
	plt.scatter(time_since_start,KangleZ, c=u'r') #print kz angle
	plt.scatter(time_since_start,gyoz, c=u'b', marker=u'.')  #print gyro z
	plt.scatter(time_since_start,aaz, c=u'g', marker=u'x')  #print acc z
	
	plt.draw()
	
	return

#post data to console	
def output_console(i, delta_t, time_since_start,gyox,gyoy,gyoz,aax,aay,aaz,KangleX,KangleY,KangleZ):
	print ("interval", i)
	print("delta_t: ", delta_t, "time_since_start: ", time_since_start)
	print("gyro angles: x-",gyox," y-",gyoy," z-",gyoz)
	print("Acc angles:  x-",aax," y-",aay," z-",aaz)
	print("KF angles:   x-", KangleX," y-", KangleY," z-", KangleZ)
	print("delta_t: ", delta_t, "time_since_start: ", time_since_start)
	return

def removeG(a, g): #i modified this part to only change X and Y, Z remains unchanged because it is just gravity...
	if not a[0]==0: 
	    a[0] = a[0] - g[0]
	if not a[1]==0:
	    a[1] = a[1] - g[1]
	a[2] = a[2] - g[2]
	return a
	
def pbar(window, output):
       	window.clear()
	#window.addstr(0,0, "X Acceleration: "+str(output[0])+"\nX Kalman: "+str(output[1])+"\nX Kalman Rounded: "+str(output[2])+"\nY Acceleration: "+str(output[3])+"\nY Kalman: "+str(output[4])+"\nY Kalman Rounded: "+str(output[5])+"\nZ Acceleration: "+str(output[6])+"\nZ Kalman: "+str(output[7])+"\nZ Kalman Rounded: "+str(output[8]))
        window.addstr(0,0,"Z: "+str(output[7]))
	window.refresh()
        time.sleep(0.5)

def mag_compass(magx, magy): #compass without pitch and roll using magnetometer
    #compass w/o pitch and roll
    	if magy > 0:
            return (90-(math.degrees(math.atan(magx/magy))))
    	if magy < 0:
            return (270-(math.degrees(math.atan(magx/magy))))
    	if magy ==  0 and magx < 0:
            return 180
    	if magy == 0 and magx > 0:
            return 0

def get_distance_xy(distance, angle):
	#print 'distance: ' +str(distance)
	if angle == 90:
		return [0, distance]
	elif angle == 180:
		return [-distance, 0]
	elif angle == 270:
		return [0, -distance]
	elif angle == 0:
		return [distance, 0]
	else:
		if angle < 90 and angle > 0:
			angle = math.radians(angle)
			return [math.sin(angle)*distance, math.cos(angle)*distance]
		elif angle < 180 and angle > 90:
			#print 'larger than 90 less than 180'
			angle = math.radians(180-angle)
                        return [-(math.sin(angle)*distance), math.cos(angle)*distance]
		elif angle < 270 and angle > 180:
			angle = math.radians(270-angle)
                        return [-(math.sin(angle)*distance), -(math.cos(angle)*distance)]
		elif angle > 270: #angle larger than 270 but smaller than 360
			angle = math.radians(360-angle)
                        return [-(math.sin(angle)*distance), -(math.cos(angle)*distance)]


def getdata():
	print("starting main")
	#start random num gen
	random.seed()

	velocity = [0,0,0]
	distance = [0,0,0]
	gyoangle = [0,0,0]
	#pitch = 0
	#roll = 0
	#x = pitch, y = roll, z = heading

	#initialize filter
	kalmanfilter = kalmanFilter() #initialize the kalmanFilter class inside this script
	
	imu = kalmanfilter.imulib #initializing the imu class in accxbee.py
	time.sleep(0.1)
	xbee = kalmanfilter.xbeelib #initializing the xbee class in xbeelib.py
	time.sleep(0.1)
	kalmanfilter = calibrate_filter(kalmanfilter) #should calibrate everything (takes aprox 15+ seconds)
	
        #print out the value just to check if the imu returns the correct value, which the Z value should be 1
        print "x:", imu.get_acc_x()
        print "y:", imu.get_acc_y()
        print "z:", imu.get_acc_z()


	#graph initialize
	#print("starting main graph")
	#graph_init()
	
	#this is time since start of program
	timer_mark = kalmanfilter.getTimer_Mark() #get time before loop in milliseconds
	time_since_start = int(round(time.time()*1000)) - timer_mark 

	print("beginning main loop")
	#print "X Acceleration\tY Acceleration\tZ Acceleration\tX Kalman\tY Kalman\tZ Kalman\tX Kalman Rounded\tY Kalman Rounded\tZ Kalman Rounded"
	
	#below are the header for printing the acceleration, comment out if printing footsteps
	#print "X Acceleration,X Kalman,X Kalman Rounded,Y Acceleration,Y Kalman,Y Kalman Rounded,Z Acceleration,Z Kalman,Z Kalman Rounded"
	acclist = [0,0,0,0,0,0,0,0,0]
	i=0 #just for the loop
	#curses.wrapper(pbar)
	output = str()
	#zaverage = 0 
	footstep = 0
	footstep_flag = False
	len_of_vector_total = 0 #culmulate the length of vector to calculate the average value
	total_heading = 0
	#print 'starting the listener thread'
	#thread.start_new_thread(xbee.RX(), ())
	#try:
		#thread.start_new_thread(xbee.RX(), ())
		#xbee.RX()
	#	print 'thread started'
	#except Exception as e:
	#	print 'error starting the Receiving Thread!!!'
	#	print e.args
	#	print e
	#	sys.exit()

	print 'infinite while loop for displaying data begins'
	while(True):
		timer_mark_loop = float(round(time.time()*1000))
                timer_mark = kalmanfilter.getTimer_Mark() #get time before loop in milliseconds
                time_since_start = int(round(time.time()*1000)) - timer_mark
                time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
                #print ("interval", i)
                timer = time.time() #get time before loop in milliseconds
                #print('timer: ', round(timer) )
		#delta_t = int(round(time.time()*1000)) - timer #calculates delta time since first loop


		#time.sleep(0.02)
		gyo = kalmanfilter.calibrateGyro( imu.get_gyo_all() )

		accx = imu.get_acc_x()
        	accy = imu.get_acc_y()
        	accz = imu.get_acc_z()
        	gyox = gyo[0]
        	gyoy = gyo[1]
        	gyoz = gyo[2]
		magx = imu.get_mag_x()
		magy = imu.get_mag_y()
		magz = imu.get_mag_z()

		
		#print("delta_t: ", delta_t, "time_since_start: ", time_since_start)


		
		#updateSensors()

		#calculate time since start
		#time_since_start = int(round(time.time()*1000)) - timer_mark 
		
		aax = float(AAX(accx,accy,accz)) #x
		aaz = float(AAZ(accx,accy,accz)) #z
		aay = float(AAY(accx,accy,accz)) #y
		kalmanfilter.setPitch ( findPitch(accx,accy,accz) )
		kalmanfilter.setRoll( findRoll(accx,accy,accz) )
		#print 'pitch:'+str(math.degrees(kalmanfilter.pitch))+"   roll:"+str(math.degrees(kalmanfilter.roll))+"   heading:"+
		#str( findHeading(kalmanfilter.roll, kalmanfilter.pitch,magx , magy, magz) )
		#current magetometer angle, use for comparison
                angle = mag_compass(imu.get_mag_x(), imu.get_mag_y())

		acc2 = [aax,aay,aaz]		
		#calculate angles
		#time.sleep(.001)
		#print("accelerometer angle: x-",aax," y-",aay," z-",aaz) #for some reason if you comment out this code, everything dies
		#delta_t = int(round(time.time()*1000)) - timer #reset dt each calculation
		delta_t = kalmanfilter.delta_t
		KangleX = kalmanfilter.X(aax, gyox, delta_t) #gets calculated x
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleY = kalmanfilter.Y(aay, gyoy, delta_t) #gets calculated y
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleZ = kalmanfilter.Z(aaz, gyoz, delta_t) #gets calculated z
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		orientation = [rad_to_angle(KangleX), rad_to_angle(KangleY), rad_to_angle(KangleZ)] #doesnt need to be average because the filter already does it
		gyoangle[0] += gyox*delta_t
		gyoangle[1] += gyoy*delta_t
		gyoangle[2] += gyoz*delta_t

		#calcuate accelerations
		acceleration = [accx,accy,accz]#puts acceleration into a list
		#print ("Actual  accel",acceleration)
		#print "Get Gravity: ", kalmanfilter.getG()
		kalmanfilter.updateG() #updates g using k angles stored in object
		acceleration = removeG(acceleration, kalmanfilter.getG()) #fixes acceleration by removing gravity acceleration
		
				#round acceleration values
		acceleration = [round(acceleration[0],2),round(acceleration[1],2),round(acceleration[2],2)]
		
		#calculate distance from acceleration - g
		#distance calculations
		distance, velocity = kalmanfilter.calcDistance(distance, orientation, acceleration, velocity, delta_t)
		
		#xbee.setDistance(distance)
		#angle = mag_compass(imu.get_mag_x(), imu.get_mag_y())
		#xbee.setMagAngle(angle)
		#friction on the velocity, this is to help remove data error
		if(acceleration[0] == 0): #if accel = 0 remove .5 of velocity
			if(round(velocity[0],3) != 0):
				velocity[0] = velocity[0] * 0.5
				#velocity[0] = round(velocity[0],3) #can't have this rounded or velocity gets stuck at 0.001 at the end
		if(acceleration[1] == 0): #if accel = 0 remove .5 of velocity
			if(round(velocity[1],3) != 0):
				velocity[1] = velocity[1] * 0.5
		if(acceleration[2] == 0): #if accel = 0 remove .5 of velocity
			if(round(velocity[2],3) != 0):
				velocity[2] = velocity[2] * 0.5
		
		
		
		
		#print ("RemoveG accel", acceleration)
		#print ( "%0.3f" % acceleration[0],"%0.3f" % acceleration[1],"%0.3f" % acceleration[2])
		
		#uncomment this to print distance data
		#print ("Delta   T (ms)   ", delta_t, "S")
		#print ("Total   T (ms)   ", time_since_loop_start)
		#print ("Rounded A (g)    ", "%0.6f" % acceleration[0],"%0.6f" % acceleration[1],"%0.6f" % acceleration[2]) #just for our benefit
		#print ("Rounded V (g*s)  ", "%0.6f" % velocity[0],"%0.6f" % velocity[1],"%0.3f" % velocity[2])
		#print ("Rounded D (g*s^2)", "%0.6f" % distance[0],"%0.6f" % distance[1],"%0.6f" % distance[2])
		#print ("---")
		


		#print ("RemoveG accel", acceleration)
		#print ("Rounded accel", "%0.3f" % acceleration[0],"%0.3f" % acceleration[1],"%0.3f" % acceleration[2]) #just for our benefit
		#output = accx,accy,accz,acceleration[0],acceleration[1],acceleration[2],"%0.3f" % acceleration[0],"%0.3f" % acceleration[1],"%0.3f" % acceleration[2]
		#output = accx,acceleration[0],"%0.3f" % acceleration[0],accy,acceleration[1], "%0.3f" % acceleration[1],accz,acceleration[2],"%0.3f" % acceleration[2]
		#output = "X Acceleration: "+str(accx)+"\nX Kalman: "+str(acceleration[0])+"\nY Acceleration: "+str(accy)+"\nY Kalman: "+str(acceleration[1])+"\nZ Acceleration: "+str(accz)+"\nZ Kalman: "+str(acceleration[2])
		#output = "Z Acceleration: "+str(accz)
		#acclist = [0,0,0,0,0,0,0,0,0]
		acclist[0]=accx
		acclist[1]=acceleration[0]
		acclist[2]="%0.3f" % acceleration[0]
		acclist[3]=accy
		acclist[4]=acceleration[1]
		acclist[5]="%0.3f" % acceleration[1]
		acclist[6]=accz
		acclist[7]=acceleration[2]
		acclist[8]="%0.3f" % acceleration[2]
		#print output

		#print ("kalman orientation: ", orientation)
		#print( "accelerometer angles", acc2[2]*RAD_TO_DEG%360)
		#output_console(i, delta_t, time_since_start,gyox,gyoy,gyoz,aax,aay,aaz,KangleX,KangleY,KangleZ)
		#graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
		total_heading += findHeading(kalmanfilter.roll, kalmanfilter.pitch, accx, accy, accz)
		i+=1	#remove this eventually because FILTER IS ETERNAL BWAAHAHHAHWAH
		if i % 5 == 0:
			print ('gyro with calibration: ', gyoz)
			print ('gyro angle: ',  gyoangle[2])
			#print ('heading: ', findHeading(KangleX, KangleY, accx, accy, accz))
			#print ('average heading: ', total_heading/5)
			print ('KangleZ: ', KangleZ)
			total_heading = 0
		#curses.wrapper(pbar, acclist)
		
		#this will be replaced by the Kalman Z angle eventually		
		xbee.setAngle(KangleZ)
		#print ('magnetometer orientation: ',angle )
		#instead of only measuring the Z axis, we use the length of vector to find the total acceleration in case the IMU is not only facing one direction
		len_of_vector = float(math.sqrt(accx*accx+accy*accy+accz*accz))
		len_of_vector_total += len_of_vector


		#if the length of vector is larger than the average length of vector
		if footstep_flag is False and len_of_vector > (len_of_vector_total / i * footstep_multiplier):
			footstep_flag = True #set a flag that indicates footstep was counted
			footstep += 1
			#print '%s:%s:%s:%s' % (now.hour, now.minute, now.second, now.microsecond) + "\t"+str(len_of_vector)+ "\t"+str(angle) + "\t" + str(footstep)+ "\t" + str(KangleX) + "\t" + str(KangleY) + "\t" + str(KangleZ)			
			#uncomment this to use as xbee transmission

			print 'attempting to send'
			#replace xbee.mag_angle with rad_to_angle(KangleZ) when using kalmanfilter
			distance_xy = get_distance_xy(footstep_length(1.75), xbee.mag_angle) #first value is distance in meter, second is angle
			#print str(distance_xy)
			print 'step: '+str(footstep)
			xbee.setDistanceStep(distance_xy)
			print 'distance before summation'+str(distance_xy)+ "   "
			print 'distance after summation'+str(xbee.distance_step[0])+"\t"+str(xbee.distance_step[1])+"\t"
			#xbee.TX("pedometer:"+str(xbee.distance_step[0])+"\t"+str(xbee.distance_step[1])+"\t"+str(rad_to_angle(KangleZ))+";")
			#xbee.TX("orientation:"+str(KangleX)+"\t"+str(KangleY)+"\t"+str(KangleZ)+";")
			print 'kalman orientation: '+str(rad_to_angle(KangleX))+'\t'+str(rad_to_angle(KangleY))+'\t'+str(rad_to_angle(KangleZ))
			#print ('mag heading: ', findHeading(kalmanfilter.roll, kalmanfilter.pitch,magx , magy, magz))
			#print ('mag angle: ', xbee.mag_angle)
			#print ("Rounded D (g*s^2)", "%0.6f" % distance[0],"%0.6f" % distance[1],"%0.6f" % distance[2])	
			#xbee.TX("mag_angle:"+str(rad_to_angle(KangleZ))+";")
			#start the timer on the other end
			#xbee.TX("!")
			#xbee.TX("?")
			#stop the timer on the other end
			
			#time.sleep(0.5)
			print 'send finished'

		elif footstep_flag is True and len_of_vector <= (len_of_vector_total / i * footstep_multiplier):
			footstep_flag = False #removed the footstep flag for the next footstep detection
		while time.time() - timer < 0.02:
			time.sleep(0.001)
		#print ('time.time(): ' , round( time.time()) ) #message to see if each loop is 20 ms
		#print '%s:%s:%s:%s' % (now.hour, now.minute, now.second, now.microsecond) + "\t"+str(len_of_vector)+ "\t"+str(angle) + "\t" + str(footstep)+ "\t" + str(KangleX) + "\t" + str(KangleY) + "\t" + str(KangleZ) 
		#print '%s:%s:%s:%s' % (now.hour, now.minute, now.second, now.microsecond) + "\t"+str(zaverage/5)+ "\t"+str(angle)
		
				

	print("end of main loop")
	#time.sleep(10)
	#print("closing main graph")
	#plt.close("all")
	#plt.ioff()
		
def main():
	thread.start_new_thread(getdata(), ())        

if __name__=='__main__': main()
    

