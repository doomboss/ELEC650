import serial, usb.core, usb.util, thread, time, sys
#from xbee import XBee
SERIALPORT = "/dev/ttyUSB1"
BAUDRATE = 9600
DESTINATION = '\xFF\xFF' # Module 1: \x0A\xFB, Module 2: \x0A\xEF, All Modules: \xFF\xFF



ser = serial.Serial(SERIALPORT, BAUDRATE)

#xb = XBee(ser)

def RX():
	ser_rx = ""#Buffer for receiving complete messages, messages should begin with the keyword 'START/' and end with the 	keyword '/END'. Everything sent that is not between these keywords will be filtered out.
	while True:
			try:
				received = ser.read()
				if received == ";":
					print ser_rx
					din=ser_rx
					ser_rx = ""
				else:
					ser_rx += str(received)
			except:
				print('Error_RX')
				break

def TX():
	#TX_test = "0"
	inp = ""
	while True:
			try:
				#if inp!=TX_test:
					#TX_test = inp
					#ser.write(inp)
				inp=raw_input()
				ser.write(inp)
				#else:
					#continue
			except:
				print('Error_TX')
				break

print 'Starting...'

ser.write("\r\n\r\n\r\n\r\n")
ser.write("B")
thread.start_new_thread(RX, ())
thread.start_new_thread(TX, ())
ser.write("Connection Established")

while True:
		try:
			x=0
		except:
			print('Error')
	
print 'Ended...'
