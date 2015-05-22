import math #used for atan
import time #used for delta_t
import matplotlib.pyplot as plt #used for graphing
import random #for creating noise

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
	accx = float(0)
	accy = float(0)
	accz = float(-1)
	gyox = float(0)
	gyoy = float(0)
	gyoz = float(0)
	
	count = 0
	
	while (time_since_start < 7000): #stop after 7 seconds
		#calculate time since start
		time_since_start = float(round(time.time()*1000) - timer_mark )
		#time since loop start
		time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
		#calculate delta_t
		timer = float(round(time.time()*1000)) #get time before loop in milliseconds
		
		#get angles
		#update sensors()
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
	#a[2] = a[2] - g[2]
	return a
	


def main():
	print("starting main")
    #start random num gen
	random.seed()
	
	#x = pitch, y = roll, z = heading
	
	#set constants for testing
	accx = 0 #randomuniform
	accy = 0
	accz = -1
	gyox = 0
	gyoy = 0
	gyoz = 0
	
	#initialize filter
	kalmanfilter = kalmanFilter() #initialize the filter
	kalmanfilter = calibrate_filter(kalmanfilter) #should calibrate everything (takes aprox 15+ seconds)
	
	#graph initialize
	print("starting main graph")
	graph_init()
	
	#this is time since start of program
	timer_mark = kalmanfilter.getTimer_Mark() #get time before loop in milliseconds
	time_since_start = int(round(time.time()*1000)) - timer_mark 
	
	print("beginning main loop")
	i=0 #just for the loop 
	while i<100:
		#print ("interval", i)
		timer = int(round(time.time()*1000)) #get time before loop in milliseconds
		delta_t = int(round(time.time()*1000)) - timer #calculates delta time since first loop
	
		#print("delta_t: ", delta_t, "time_since_start: ", time_since_start)


		
		#updateSensors()
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
		delta_t = int(round(time.time()*1000)) - timer #dt
		KangleY = kalmanfilter.Y(aay, gyoy, delta_t) #gets calculated y
		delta_t = int(round(time.time()*1000)) - timer #dt
		KangleZ = kalmanfilter.Z(aaz, gyoz, delta_t) #gets calculated y
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		
		#calcuate accelerations
		acceleration = [accx,accy,accz]#puts acceleration into a list
		print ("Actual  accel",acceleration)
		kalmanfilter.updateG() #updates g using k angles stored in object
		acceleration = removeG(acceleration, kalmanfilter.getG()) #fixes acceleration by removing gravity acceleration
		print ("RemoveG accel", acceleration)
		print ("Rounded accel", "%0.3f" % acceleration[0],"%0.3f" % acceleration[1],"%0.3f" % acceleration[2]) #just for our benefit
		
		#store orientation
		orientation = [KangleX, KangleY, KangleZ]

		#output_console(i, delta_t, time_since_start,gyox,gyoy,gyoz,aax,aay,aaz,KangleX,KangleY,KangleZ)
		graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
		
		#time.sleep(.5)  #delay
		i+=1		#remove this eventually because FILTER IS ETERNAL BWAAHAHHAHWAH
	
	print("end of main loop")
	time.sleep(10)
	print("closing main graph")
	plt.close("all")
	plt.ioff()
		
        

if __name__=='__main__': main()
    

