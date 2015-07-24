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
		self.datalist = list()
		time.sleep(0.1)
		self.ser.write('\r\n\r\n\r\n\r\n')
		time.sleep(0.5)
		self.ser.write('B')
		self.imulib = imulib
		self.timeholder = 0
		time.sleep(0.3)
		self.start()

	def setIMU(self, imulib):
		self.imulib = imulib

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
					else:
						print ser_rx
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
			self.ser.write(str(data))
		except KeyboardInterrupt:
			pass
			sys.exit()
		#except Exception as exception:
			#print('Error_TX') print(exception.args) break
