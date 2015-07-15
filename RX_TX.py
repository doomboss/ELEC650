import serial, usb.core, usb.util, thread, time, sys 
SERIALPORT = "/dev/ttyUSB0" 
BAUDRATE = 9600 
DESTINATION = '\xFF\xFF' # Module 1: \x0A\xFB, Module 2: \x0A\xEF, All Modules: \xFF\xFF 
ser = serial.Serial(SERIALPORT, BAUDRATE) 
 
BUF_rx = list() #Buffer for receiving complete messages, messages should begin with the keyword 'START/' and end with the keyword '/END'. Everything sent that is not between these keywords will be filtered out. 
def RX():
	ser_rx = ""
	while True:
		try:
			received = ser.read()
			if received == ";":
				print ser_rx
				ser_rx = ""
			else:
				ser_rx += str(received)
				#print(ser_rx) i=0 BUF_rx[:i] + ser_rx i+=1
		except KeyboardInterrupt:
			pass
		#except Exception as exception:
		#print('Error_RX') print(exception.args) break
def TX():
	while True:
		try:
			inp = raw_input()
			ser.write(inp)
		except KeyboardInterrupt:
			pass
		#except Exception as exception:
		#print('Error_TX') print(exception.args) break
print 'Starting...' 
ser.write("\r\n\r\n\r\n\r\n")
ser.write("B") 
ser.write("Connection Established")
thread.start_new_thread(RX, ()) 
thread.start_new_thread(TX, ()) 
while True:
	try:
		x=0
	except:
		print('Error')
	
print 'Ended...'
