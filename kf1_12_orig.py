import math #used for atan
import time #used for delta_t
import matplotlib.pyplot as plt #used for graphing
import random #for creating noise
from msvcrt import getch #keypresses for testing

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
		self.KFangleX = float(0.0)
		self.KFangleY = float(0.0)
		self.KFangleZ = float(0.0)
		self.gmag = float(0)
		self.g = [0,0,0] #holds xyz of grav
		self.timer_mark = float(int(round(time.time()*1000))) #get time at initialization in milliseconds (used only for graphs)
		
	def getOrientation(self):
		return [self.KFangleX, self.KFangleY, self.KFangleZ]
		
	def setgmag(self, grav):
		self.gmag = grav		
		return #don't need to return anything
	
	def updateG(self):
		#sets the g array
		self.g[0] = math.cos(self.KFangleX) * self.gmag
		self.g[1] = math.cos(self.KFangleY) * self.gmag
		self.g[2] = math.cos(self.KFangleZ) * self.gmag
		return
	
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
		d[2] = v[2] + (v[2]*t)/1000 + ((a[2]*t*t)/2)/(1000*1000)
		
		return  d, v
		
	
    #Y filter call
	#accAngle = angle measured with atan2 using the accelerometer
	#gyroRate = angle measured using the gyroRate
	#looptime = loop time in millis()
	def Y(self, accAngle, gyroRate, looptime):
		#correct theangle
		#accAngle = anglecorrection(accAngle)
	
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
		#correct the angle
		accAngle = anglecorrection(accAngle)
	
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
		#correct theangle
		accAngle = anglecorrection(accAngle)
		
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
		
#---------------

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
	
	#constants for testing
	accel, gyro = updateSensors()
	accx = accel[0]
	accy = accel[1]
	accz = accel[2]
	gyox = gyro[0]
	gyoy = gyro[1]
	gyoz = gyro[2]
	
	count = 0
	
	#generally overshooting the calibration filter time improves the data long term, but it takes longer to start
	while (time_since_start < 15000): #stop after 15 seconds
		#calculate time since start
		time_since_start = float(round(time.time()*1000) - timer_mark )
		#time since loop start
		time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
		#calculate delta_t
		timer = float(round(time.time()*1000)) #get time before loop in milliseconds
		
		#get angles
		accel, gyro = updateSensors()
		accx = accel[0]
		accy = accel[1]
		accz = accel[2]
		gyox = gyro[0]
		gyoy = gyro[1]
		gyoz = gyro[2]
	
		#calculate acceleration angles
		aax = float(AAX(accx,accy,accz)) #x
		aaz = float(AAZ(accx,accy,accz)) #z
		aay = float(AAY(accx,accy,accz)) #y
		
		#calculating alpha which is for averaging in the acceleration
		time.sleep(.01) #we need a sleep here because otherwise the program runs too fast
		delta_t = float(round(time.time()*1000) - timer) #calculates delta time since start of a loop
		alpha = float(time_since_loop_start / (time_since_loop_start + delta_t)) #this calculates a ratio used for averaging over time
		
		#average in acceleration values
		acc[0] = alpha*acc[0]+(1-alpha)*accx
		acc[1] = alpha*acc[1]+(1-alpha)*accy
		acc[2] = alpha*acc[2]+(1-alpha)*accz
		
		#print("accelerometer averages", acc)
		
		#print("accelerometer angle: x-",aax," y-",aay," z-",aaz) #for some reason if you comment out this code, everything dies

		delta_t = int(round(time.time()*1000)) - timer #reset dt each calculation
		KangleX = kf.X(aax, gyox, delta_t) #gets calculated x
		delta_t = int(round(time.time()*1000)) - timer #dt
		KangleY = kf.Y(aay, gyoy, delta_t) #gets calculated y
		delta_t = int(round(time.time()*1000)) - timer #dt
		KangleZ = kf.Z(aaz, gyoz, delta_t) #gets calculated y
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		
		orientation = [KangleX, KangleY, KangleZ] #doesnt need to be average because the filter already does it
		
		#calibration graph
		#graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
	
	#calibration completed, orientation should be correct
	#important!!! setting gravity should only happen once
	mag = calcMag(acc) #magnitude of gravity vector
	kf.setgmag(mag) #set the kalman filter grav
	print (mag)
	print (orientation)
	kf.updateG() #updates the component vectors of g from the magnitude and angles
	'''
	print("orientation",orientation)
	print("acceleration",acc)
	print("acc-g",removeG(acc, kf.getG()))
	'''
	print("end of calibration")
	
	#close calibration graph and sleep for a bit
	
	#return the calibrated filter
	print("sleeping")
	time.sleep(3) #remove this later
	print("clearing graph")
	#plt.clf() #clears figure
	return kf
	
	
		
	
	
#angle correction when it hits 180 (used in kf object)
def anglecorrection(angle):
	if angle >0:
		return (angle)
	else:
		return (180)	

def calcMag(m):
	return float(math.sqrt(m[0]*m[0]+m[1]*m[1]+m[2]*m[2]))

def AAX(accx, accy, accz):
	return math.acos(accx/calcMag([accx,accy,accz])) 

def AAZ(accx, accy, accz):
	return math.acos(accz/calcMag([accx,accy,accz]))

def AAY(accx, accy, accz):
	return math.acos(accy/calcMag([accx,accy,accz]))

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
	
def graph_init_distance():
    #set up graph stuff
	plt.ion() #activates interactive plots
	plt.show() #shows the graph
	#plt.suptitle("Calibration routine")

	#labels
	#x
	#plt.subplot(3,1,1)
	plt.xlabel("x (meters)")
	plt.ylabel("y (meters)")
	plt.title("Location")
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

def graph_update_distance(distance):
	#x
	#plt.subplot(3,1,1)
	plt.scatter(distance[0],distance[1], c=u'r') #print kx angle

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

def removeG(a, g):
	a[0] = a[0] - g[0]
	a[1] = a[1] - g[1]
	a[2] = a[2] - g[2]
	return a
	
def updateSensors():
	#set constants for testing
	#x = pitch, y = roll, z = heading
	'''
	twos_comp_combine_acc(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB), acctranslate)
    accy = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_Y_MSB), b.read_byte_data(LSM, ACC_Y_LSB), acctranslate)
    accz = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_Z_MSB), b.read_byte_data(LSM, ACC_Z_LSB), acctranslate)
	
	accel = [twos_comp_combine_acc(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB), acctranslate),0,-1]
	'''
	accel = [0,0,-1]
	gyro = [0,0,0]
	return accel, gyro

def main():
	print("starting main")
    #start random num gen
	random.seed()
	
	#pull in values
	accel,gyro = updateSensors()
	accx = accel[0]
	accy = accel[1]
	accz = accel[2]
	gyox = gyro[0]
	gyoy = gyro[1]
	gyoz = gyro[2]
	
	velocity = [0,0,0]
	distance = [0,0,0]
	
	#initialize filter
	kalmanfilter = kalmanFilter() #initialize the filter
	kalmanfilter = calibrate_filter(kalmanfilter) #should calibrate everything (takes aprox 15+ seconds)
	orientation = kalmanfilter.getOrientation() #pulls the most recent orientation out of the filter, after calibration
	
	#graph initialize
	print("starting main graph")
	graph_init_distance()
	
	#this is time since start of program
	timer_mark = kalmanfilter.getTimer_Mark() #get time before loop in milliseconds
	timer_mark_loop = float(round(time.time()*1000))
	time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
	time_since_start = float(round(time.time()*1000) - timer_mark)
	
	print("beginning main loop")
	i=0 #just for the loop 
	while i<70:
		#print ("interval", i)
		
		time_since_start = int(round(time.time()*1000)) - timer_mark 
		time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
		timer = int(round(time.time()*1000)) #get time before loop in milliseconds
		delta_t = int(round(time.time()*1000)) - timer #calculates delta time since first loop
	
		#print("delta_t: ", delta_t, "time_since_start: ", time_since_start)


		
		#update Sensors 
		accel,gyro = updateSensors()
		
		#key presses for sensors
		'''
		if i == 10:
			accy = -1
			
		if i == 20:
			accy = 0
			accx = 1
			print ("x = 1")
		
		if i == 30:
			accx = 0
			accy = +1
			
			print ("y = -1")
		
		if i == 50:
			accx = -1
			accy = 0
			
			print ("x = -1")
		'''
			
		accx = accel[0]
		accy = accel[1]
		accz = accel[2]
		gyox = gyro[0]
		gyoy = gyro[1]
		gyoz = gyro[2]
		
		#accelerometer angles in degrees

		'''
		#with random
		gyox += random.randint(-3,3)
		gyoy += random.randint(-3,3)
		gyoz += random.randint(-3,3)
		
		accx = 2 + random.uniform(-0.1,0.1)
		accy = 3 + random.uniform(-0.1,0.1)
		accz = 4 + random.uniform(-0.1,0.1)
		'''

		#without random
		
		#calculating alpha which is for averaging in the acceleration
		time.sleep(.01) #we need a sleep here because otherwise the program runs too fast
		delta_t = float(round(time.time()*1000) - timer) #calculates delta time since start of a loop
		alpha = float(time_since_loop_start / (time_since_loop_start + delta_t)) #this calculates a ratio used for averaging over time

		
		#calculate time since start
		time_since_start = int(round(time.time()*1000)) - timer_mark 
		
		#print("gyro angle:          x-",gyox," y-",gyoy," z-",gyoz)
		aax = float(AAX(accx,accy,accz)) #x
		aaz = float(AAZ(accx,accy,accz)) #z
		aay = float(AAY(accx,accy,accz)) #y
		
		#calculate angles
		#time.sleep(.001)
		#print("accelerometer angle: x-",aax," y-",aay," z-",aaz) #for some reason if you comment out this code, everything dies
		delta_t = int(round(time.time()*1000)) - timer #reset dt each calculation
		KangleX = kalmanfilter.X(aax, gyox, delta_t) #gets calculated x
		KangleY = kalmanfilter.Y(aay, gyoy, delta_t) #gets calculated y
		KangleZ = kalmanfilter.Z(aaz, gyoz, delta_t) #gets calculated y
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		
		#update orientation array to pass to object
		orientation = [KangleX, KangleY, KangleZ]
		
		#calculate accelerations
		acceleration = [accx,accy,accz]#puts acceleration into a list
		#print ("Actual  accel",acceleration)
		kalmanfilter.updateG() #updates g using k angles stored in object
		acceleration = removeG(acceleration, kalmanfilter.getG()) #fixes acceleration by removing gravity acceleration
		
		#round acceleration values
		acceleration = [round(acceleration[0],2),round(acceleration[1],2),round(acceleration[2],2)]
		
		#calculate distance from acceleration - g
		#distance calculations
		distance, velocity = kalmanfilter.calcDistance(distance, orientation, acceleration, velocity, delta_t)
		
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
		print ("Delta   T (ms)   ", delta_t, "S")
		print ("Total   T (ms)   ", time_since_loop_start)
		print ("Rounded A (g)    ", "%0.6f" % acceleration[0],"%0.6f" % acceleration[1],"%0.6f" % acceleration[2]) #just for our benefit
		print ("Rounded V (g*s)  ", "%0.6f" % velocity[0],"%0.6f" % velocity[1],"%0.3f" % velocity[2])
		print ("Rounded D (g*s^2)", "%0.6f" % distance[0],"%0.6f" % distance[1],"%0.6f" % distance[2])
		print ("---")
		
		#draw graph
		graph_update_distance(distance)
		
		#store orientation
		orientation = [KangleX, KangleY, KangleZ]

		#output_console(i, delta_t, time_since_start,gyox,gyoy,gyoz,aax,aay,aaz,KangleX,KangleY,KangleZ)
		#graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
		
		#time.sleep(.5)  #delay
		i+=1		#remove this eventually because FILTER IS ETERNAL BWAAHAHHAHWAH
	
	print("end of main loop")
	time.sleep(10)
	print("closing main graph")
	plt.close("all")
	plt.ioff()
		
        

if __name__=='__main__': main()
    

