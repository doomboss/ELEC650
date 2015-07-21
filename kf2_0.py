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
RAD_TO_DEG = 57.29578
footstep_multiplier = 2 #indicates the value that used for detecting the footstep 

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
		self.imulib = accxbee.IMU() #initialize the accxbee script and the imu class, for getting value from the IMU module
		self.xbeelib = xbeelib.XBEE(self.imulib) #initialize the xbeelib script and the xbee class, for the XBEE transmit and receive
		#xbeelib.setIMU(imulib)

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
	count = 0
	
	while (time_since_start < 20000): #stop after 7 seconds
		#calculate time since start
		time_since_start = float(round(time.time()*1000) - timer_mark )
		#time since loop start
		time_since_loop_start = float(round(time.time()*1000) - timer_mark_loop) #start of first loop
		#calculate delta_t
		timer = float(round(time.time()*1000)) #get time before loop in milliseconds
		
		#use kf.imulib to access the accxbee.py for getting the imu data
		#kf is KalmanFilter class
		accx = kf.imulib.get_acc_x()
		accy = kf.imulib.get_acc_y()
		accz = kf.imulib.get_acc_z()
		gyox = kf.imulib.get_gyo_x()
		gyoy = kf.imulib.get_gyo_x()
		gyoz = kf.imulib.get_gyo_x()
		
		#get angles
		#update sensors()
		#calculate acceleration angles
		aax = float(AAX(accx,accy,accz)) #x
		aaz = float(AAZ(accx,accy,accz)) #z
		aay = float(AAY(accx,accy,accz)) #y
		
		#calculating alpha which is for averaging in the acceleration
		while (float(round(time.time()*1000))- time_since_loop_start < 20):
			#print "sleeping to meet 20ms DT"
			time.sleep(.001) #we need a sleep here because otherwise the program runs too fast
		#print float(round(time.time()*1000))- time_since_loop_start
		
		delta_t = float(round(time.time()*1000) - timer) #calculates delta time since start of a loop
		'''
		alpha = float(time_since_loop_start / (time_since_loop_start + delta_t)) #this calculates a ratio used for averaging over time

		#average in acceleration values
		acc[0] = alpha*acc[0]+(1-alpha)*accx
		acc[1] = alpha*acc[1]+(1-alpha)*accy
		acc[2] = alpha*acc[2]+(1-alpha)*accz
		'''
		#sum of all accelerations
		acc[0] = acc[0]+accx
		acc[1] = acc[1]+accy
		acc[2] = acc[2]+accz
		count+=1
		
		#print("accelerometer averages", acc)
		
		#print("accelerometer angle: x-",aax," y-",aay," z-",aaz) #for some reason if you comment out this code, everything dies

		#delta_t = int(round(time.time()*1000)) - timer #reset dt each calculation
		KangleX = kf.X(aax, gyox, delta_t) #gets calculated x
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleY = kf.Y(aay, gyoy, delta_t) #gets calculated y
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleZ = kf.Z(aaz, gyoz, delta_t) #gets calculated y
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		
		orientation = [KangleX, KangleY, KangleZ] #doesnt need to be average because the filter already does it
		if ( (float(round(time.time()*1000)) - timer) < 20):
			time.sleep( (20 - ( float(round(time.time()*1000)) - timer))/1000 )
		#calibration graph
		#graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
	
	#calibration completed, orientation should be correct
	#important!!! setting gravity should only happen once
	#average accelerations from count
	acc[0]=acc[0]/count
	acc[1]=acc[1]/count
	acc[2]=acc[2]/count
	
	mag = calcMag(acc) #magnitude of gravity vector
	kf.setgmag(mag) #set the kalman filter grav
	kf.updateG() #updates the component vectors of g from the magnitude and angles
	'''
	print("orientation",orientation)
	print("acceleration",acc)
	print("acc-g",removeG(acc, kf.getG()))
	'''
	print("gravity magnitude", kf.getgmag())
	print("gravity removed", kf.getG())
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
    	if (magy > 0):
            return (90-(math.degrees(math.atan(magx/magy))))
    	if (magy < 0):
            return (270-(math.degrees(math.atan(magx/magy))))
    	if (magy ==  0 and magx < 0):
            return 180
    	if (magy == 0 and magx > 0):
            return 0

def getdata():
	print("starting main")
	#start random num gen
	random.seed()


	#x = pitch, y = roll, z = heading

	#initialize filter
	kalmanfilter = kalmanFilter() #initialize the kalmanFilter class inside this script
	
	imu = kalmanfilter.imulib #initializing the imu class in accxbee.py
	time.sleep(0.1)
	xbee = kalmanfilter.xbeelib #initializing the xbee class in xbeelib.py
	time.sleep(0.1)
	#kalmanfilter = calibrate_filter(kalmanfilter) #should calibrate everything (takes aprox 15+ seconds)
	
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
	zaverage = 0 
	footstep = 0
	footstep_flag = False
	len_of_vector_total = 0 #culmulate the length of vector to calculate the average value
	
	print 'starting the listener thread'
	#thread.start_new_thread(xbee.RX(), ())
	try:
		thread.start_new_thread(xbee.RX(), ())
		print 'thread started'
	except KeyboardInterrupt as ki:
		print 'keyboardinterrupt'
		sys.exit()
	except Exception as e:
		print 'error starting the Receiving Thread!!!'
		print e.args
		print e
		sys.exit()

	while(True):
		accx = imu.get_acc_x()
        	accy = imu.get_acc_y()
        	accz = imu.get_acc_z()
        	gyox = imu.get_gyo_x()
        	gyoy = imu.get_gyo_y()
        	gyoz = imu.get_gyo_z()


		#print ("interval", i)
		timer = int(round(time.time()*1000)) #get time before loop in milliseconds
		delta_t = int(round(time.time()*1000)) - timer #calculates delta time since first loop
	
		#print("delta_t: ", delta_t, "time_since_start: ", time_since_start)


		
		#updateSensors()
		#accelerometer angles in degrees

		'''
		#with random
		gyox += imu.get_gyo_x()
		gyoy += imu.get_gyo_y()
		gyoz += imu.get_gyo_z()
		
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
		acc2 = [aax,aay,aaz]		
		#calculate angles
		#time.sleep(.001)
		#print("accelerometer angle: x-",aax," y-",aay," z-",aaz) #for some reason if you comment out this code, everything dies
		#delta_t = int(round(time.time()*1000)) - timer #reset dt each calculation
		KangleX = kalmanfilter.X(aax, gyox, delta_t) #gets calculated x
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleY = kalmanfilter.Y(aay, gyoy, delta_t) #gets calculated y
		#delta_t = int(round(time.time()*1000)) - timer #dt
		KangleZ = kalmanfilter.Z(aaz, gyoz, delta_t) #gets calculated y
		#print("Kalman angles:       x-", KangleX," y-", KangleY," z-", KangleZ)
		
		#calcuate accelerations
		acceleration = [accx,accy,accz]#puts acceleration into a list
		#print ("Actual  accel",acceleration)
		#print "Get Gravity: ", kalmanfilter.getG()
		kalmanfilter.updateG() #updates g using k angles stored in object
		acceleration = removeG(acceleration, kalmanfilter.getG()) #fixes acceleration by removing gravity acceleration
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
		zaverage = zaverage + accz
		#store orientation
		orientation = [KangleX, KangleY, KangleZ]
		#print ("kalman orientation", orientation)
		#print( "accelerometer angles", acc2[2]*RAD_TO_DEG%360)
		#print ("gyroscope angles")
		#output_console(i, delta_t, time_since_start,gyox,gyoy,gyoz,aax,aay,aaz,KangleX,KangleY,KangleZ)
		#graph_update(time_since_start, KangleX, gyox,aax,KangleY,gyoy,aay,KangleZ,gyoz,aaz)
		
		#time.sleep(.5)  #delay
		if ( (float(round(time.time()*1000)) - timer) < 20):
                	#print float(round(time.time()*1000)) - timer, timer, float(round(time.time()*1000))
			tmp = float(20 - (float(round(time.time()*1000)) - timer) )
			#print tmp/1000
			#print str((float(round(time.time()*1000)) - timer))
			#print str(tmp/1000) 
			time.sleep( tmp/1000 )
			#time.sleep(0.02)
		#print 'after the manual delay'
		#curses.wrapper(pbar, acclist)
		#print output
		i+=1	#remove this eventually because FILTER IS ETERNAL BWAAHAHHAHWAH
		#if i % 5 == 0:
		#curses.wrapper(pbar, acclist)
		angle = mag_compass(imu.get_mag_x(), imu.get_mag_y())
		now = datetime.now()

		#instead of only measuring the Z axis, we use the length of vector to find the total acceleration in case the IMU is not only facing one direction
		len_of_vector = float(math.sqrt(accx*accx+accy*accy+accz*accz))
		len_of_vector_total += len_of_vector

		#if the length of vector is larger than the average length of vector
		if footstep_flag is False and len_of_vector > (len_of_vector_total / i * footstep_multiplier):
			footstep_flag = True #set a flag that indicates footstep was counted
			footstep += 1
			print '%s:%s:%s:%s' % (now.hour, now.minute, now.second, now.microsecond) + "\t"+str(len_of_vector)+ "\t"+str(angle) + "\t" + str(footstep)+ "\t" + str(KangleX) + "\t" + str(KangleY) + "\t" + str(KangleZ)			
			#uncomment this to use as xbee transmission
			print 'attempting to send'
			xbee.TX(str(angle)+"\t"+str(footstep)+";")
			#time.sleep(0.5)
			print 'send finished'

		elif footstep_flag is True and len_of_vector <= (len_of_vector_total / i * footstep_multiplier):
			footstep_flag = False #removed the footstep flag for the next footstep detection

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
    

