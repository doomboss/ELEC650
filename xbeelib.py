import serial, usb.core, usb.util, thread, time, sys, os
from threading import Thread
SERIALPORT = "/dev/ttyUSB" 
BAUDRATE = 9600 
DESTINATION = '\xFF\xFF' # Module 1: \x0A\xFB, Module 2: \x0A\xEF, All Modules: \xFF\xFF 

class XBEE(Thread):
	def __init__(self, imulib):
		Thread.__init__(self) 
		for x in range(0,10):
			#print SERIALPORT+str(x)
			#print os.path.exists("/dev/"+SERIALPORT+str(x))
			if os.path.exists(SERIALPORT+str(x)):
				self.ser = serial.Serial(SERIALPORT+str(x), BAUDRATE)
				break
		else:
			print 'could not find the USB port associated with XBEE... Check connection please'
			sys.exit()
		self.daemon = True	
		self.datalist = list() #used as a data holder so that whenever we want to transmit something we can just set this value and use TX to transmit
		time.sleep(0.1)
		self.ser.write('\r\n\r\n\r\n\r\n')
		self.ser.write(';')
		time.sleep(0.3)
		self.ser.write('B')
		self.ser.write(';')
		time.sleep(0.1)
		self.imulib = imulib
		self.distance = list()
		self.velocity = list()
		self.orientation = list()
		self.distance_step = [0,0]
		self.mag_angle = 0
		self.timeholder = 0
		time.sleep(0.2)
		self.start()

	def setIMU(self, imulib):
		self.imulib = imulib
	def setDistance(self, distance):
		self.distance = distance
	def setVelocity(self, velocity):
		self.velocity = velocity
	def setOrientation(self, orientation):
		self.orientation = orientation
	def setAngle(self, angle):
		self.mag_angle = angle
	def setDistanceStep(self, distance):
		self.distance_step[0] = self.distance_step[0] + distance[0]
		self.distance_step[1] = self.distance_step[1] + distance[1]


	#Buffer for receiving complete messages, messages should begin with the keyword 'START/' and end with the keyword '/END'. Everything sent that is not between these keywords will be filtered out. ser.write("\r\n\r\n\r\n\r\n")

	def run(self):
		ser_rx = ""
		while True:
			try:
				received = self.ser.read()
				#print received
				if received == ";":
					#print 'raw data: '+ser_rx
					if ser_rx == "getacc":
						#print 'getacc'
						self.TX("acc:"+str(self.imulib.get_acc_x())+"\t"+str(self.imulib.get_acc_y())+"\t"+str(self.imulib.get_acc_z())+";")
					elif ser_rx == "getgyo":
						#print 'getgyo'
						self.TX("gyo:"+str(self.imulib.get_gyo_x())+"\t"+str(self.imulib.get_gyo_y())+"\t"+str(self.imulib.get_gyo_z())+";")
					elif ser_rx == "distance":
						print 'distance to be send:'+str(self.distance)
						self.TX("distance:"+str(self.distance[0])+"\t"+str(self.distance[1])+"\t"+str(self.distance[2])+";")
					elif ser_rx == "velocity":
						self.TX("velocity:"+str(velocity[0])+"\t"+str(velocity[1])+"\t"+str(velocity[2])+";")
					elif ser_rx == "orientation":
						self.TX("orientation:"+str(angle[0])+"\t"+str(angle[1])+"\t"+str(angle[2])+";")
					elif ser_rx == "mag_angle":
						self.TX("mag_angle:"+str(self.mag_angle)+";")
					elif ser_rx == "pedometer":
						self.TX("pedometer:"+str(self.distance_step[0])+"\t"+str(self.distance_step[1])+"\t"+str(self.mag_angle)+";")
					elif ser_rx == "reset":
						self.distance_step = [0,0]
					else:
						print 'data stored: '+ser_rx
					ser_rx = ""
				#elif received == "!": #start the timer
				#	self.timeholder = time.time()
				#elif received == "?": #end the timer and calculate the time difference
				#	print time.time() - self.timeholder
				#	self.TX(time.time() - self.timeholder)
				else:
					ser_rx += str(received)
				#print(ser_rx) i=0 BUF_rx[:i] + ser_rx i+=1
			except KeyboardInterrupt:
				sys.exit()
		#except Exception as exception:
		#print('Error_RX') print(exception.args) break
	def TX(self,data):
		try:
			if not data.endswith(';'):
				data+=';'
			#print 'data to be send: '+str(data)
			self.ser.write(str(data))
		except KeyboardInterrupt:
			pass
			sys.exit()
		#except Exception as exception:
			#print('Error_TX') print(exception.args) break
