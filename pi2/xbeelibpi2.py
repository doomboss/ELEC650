import serial, usb.core, usb.util, thread, time, sys, os
from threading import Thread
import turtle
SERIALPORT = "/dev/ttyUSB" 
BAUDRATE = 9600
DESTINATION = '\xFF\xFF' # Module 1: \x0A\xFB, Module 2: \x0A\xEF, All Modules: \xFF\xFF 

class XBEE(Thread):
	print 'inside XBEE class'
	def __init__(self):
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
		self.datalist = list()
		self.ser.write('\r\n\r\n\r\n\r\n')
		time.sleep(0.2)
		self.ser.write('B')
		self.timeholder = 0
		time.sleep(0.2)
		self.ser.write(';')
		self.accdata = list()
		self.gyodata = list()
		self.distance = [float() ,float(), float()]
		self.mag_angle = 0
		self.start()


	#Buffer for receiving complete messages, messages should begin with the keyword 'START/' and end with the keyword '/END'. Everything sent that is not between these keywords will be filtered out. ser.write("\r\n\r\n\r\n\r\n")

	def run(self): # this is a receiver that continuously listen to the input from other xbee
		ser_rx = ""
		#self.TX('pedometer;')
		while True:
			try:
				received = self.ser.read()
				#print received
				if received == ";":
					#print 'raw data: '+ser_rx
					if "pedometer:" in ser_rx:
						ser_rx = ser_rx.replace("pedometer:", "")
						print 'pedometer data received: '+ser_rx
						tmplist = ser_rx.split("\t") #the result should contains x distance, y distance, and orientation angle
						self.setDistanceX( float(tmplist[0]) )
						self.setDistanceY( float(tmplist[1]) )
						self.setAngle( float(tmplist[2]) )
						#self.plot_turtle()
						
					elif "orientation:" in ser_rx:
						print 'orientation data: '
						ser_rx = ser_rx.replace("orientation:", "")
						for tmp in ser_rx.split("\t"):
							print tmp
					elif "mag_angle:" in ser_rx:
						ser_rx = ser_rx.replace("mag_angle:", "")
						print 'magnetometer angle data: '+ ser_rx
						self.mag_angle = long(ser_rx)
					elif "distance:" in ser_rx:
						print 'distance data: '
						self.distance = list()
						ser_rx = ser_rx.replace("distance:", "")
						for tmp in ser_rx.split("\t"):
							print tmp
							self.distance.append(long(tmp))
						print str(self.distance)
					elif "acc:" in ser_rx:
						#print ser_rx
						ser_rx = ser_rx.replace("acc:", "")
						self.accdata = list()
						for tmp in ser_rx.split('\t'):
							self.accdata.append(long(tmp))
					elif "gyo:" in ser_rx:
						#print ser_rx
						ser_rx = ser_rx.replace("gyo:", "")
						self.gyodata = list()
						for tmp in ser_rx.split('\t'):
							self.gyodata.append(long(tmp))
					ser_rx = ""
				#elif received == "!": #start the timer
				#	self.timeholder = time.time()
				#elif received == "?": #end the timer and calculate the time difference
				#	print time.time() - self.timeholder
					#print 299792458 * (time.time() - self.timeholder)
					#self.TX(time.time() - self.timeholder)
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
				data += ';'
			self.ser.write(str(data))
		except KeyboardInterrupt:
			pass
			sys.exit()
		#except Exception as exception:
			#print('Error_TX') print(exception.args) break

	def setDistance(self, distance):
		self.distance[0] = distance[0]
		self.distance[1] = distance[1]
	def setDistanceX(self, distance):
		self.distance[0] = distance
	def setDistanceY(self, distance):
		self.distance[1] = distance
	def setAngle(self, angle):
		self.mag_angle = angle
	
	def plot_turtle(self):

		print 'distance : '+str(self.distance[0]) + "   " + str(self.distance[0])
		#turtleplot.distance[0] = turtleplot.distance[0] + turtleplot.xbee.distance[0]
		#turtleplot.distance[1] = turtleplot.distance[1] + turtleplot.xbee.distance[1]
		#turtleplot.xbee.distance[0] = 0
		#turtleplot.xbee.distance[1] = 0
		#print 'distance counter: '+str(turtleplot.distance[0]) + "   " + str(turtleplot.distance[1])
		self.position.setheading(math.fabs(self.position.heading() - self.mag_angle))
		self.position.setx(self.distance[0]*15)
		self.position.sety(self.distance[1]*15)

