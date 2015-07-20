import serial, usb.core, usb.util, thread, time, sys, os
SERIALPORT = "/dev/ttyUSB" 
BAUDRATE = 9600 
DESTINATION = '\xFF\xFF' # Module 1: \x0A\xFB, Module 2: \x0A\xEF, All Modules: \xFF\xFF 

class xbee:
	def __init__(self, imu):
		 
		for x in range(0,10):
			if os.path.isdir("/dev/"+SERIALPORT+str(x)):
				self.ser = serial.Serial(SERIALPORT+str(x), BAUDRATE)
				break
		else:
			print 'could not find the USB port associated with XBEE... Check connection please'
			sys.exit()	
		self.datalist = list()
		time.sleep(0.1)
		self.ser.write('\r\n\r\n\r\n\r\n')
		time.sleep(0.3)
		self.ser.write('B')
		self.imulib = imu
		self.timeholder = 0
		
	def setIMU(self, imu):
		self.imulib = imu

	#Buffer for receiving complete messages, messages should begin with the keyword 'START/' and end with the keyword '/END'. Everything sent that is not between these keywords will be filtered out. ser.write("\r\n\r\n\r\n\r\n")

	def RX(self):
		ser_rx = ""
		while True:
			try:
				received = self.ser.read()
				if received == ";":
					if ser_rx == "getacc":
						TX(self.imulib.get_acc_all()+";")
					elif ser_rx == "getgyo":
						TX(self.imulib.get_gyo_all()+";")
					else:
						print ser_rx
					ser_rx = ""
				elif received == "!": #start the timer
					self.timeholder = time.now()
				elif received == "?": #end the timer and calculate the time difference
					print time.now() - self.timeholder
					
				else:
					ser_rx += str(received)
				#print(ser_rx) i=0 BUF_rx[:i] + ser_rx i+=1
			except KeyboardInterrupt:
				pass
		#except Exception as exception:
		#print('Error_RX') print(exception.args) break
	def TX(self,data):
		try:
			self.ser.write(data)
		except KeyboardInterrupt:
			pass
		#except Exception as exception:
			#print('Error_TX') print(exception.args) break
