import math #used for atan
import time #used for delta_t
import matplotlib.pyplot as plt #used for graphing
import random #for creating noise
#from msvcrt import getch #keypresses for testing
import xbeelibpi2
import sys
import turtle
from Tkinter import *


class TurtlePlot():
    #initialize
	def __init__(self):
		self.timer_mark = float(int(round(time.time()*1000))) #get time at initialization in milliseconds (used only for graphs)
		self.xbee = xbeelibpi2.XBEE()
		self.distance = [0,0,0]
		self.position = turtle.Turtle()

		#self.gui = Tk()

	def start_turtle(self):
		print("starting main graph")

		self.position.setheading(90)
		self.position.shape("arrow")

		i=0 #just for the loop 


	def plot_turtle(self):

		print 'distance from XBEE class: '+str(self.xbee.distance[0]) + "   " + str(self.xbee.distance[1])
		#turtleplot.distance[0] = turtleplot.distance[0] + turtleplot.xbee.distance[0]
		#turtleplot.distance[1] = turtleplot.distance[1] + turtleplot.xbee.distance[1]
		#turtleplot.xbee.distance[0] = 0
		#turtleplot.xbee.distance[1] = 0
		#print 'distance counter: '+str(turtleplot.distance[0]) + "   " + str(turtleplot.distance[1])

		#self.position.setheading(math.fabs(self.position.heading() - self.xbee.mag_angle))
		self.position.setheading(self.xbee.mag_angle)
		self.position.setpos(self.xbee.distance[0]*15, self.xbee.distance[1]*15)
		#self.position.setx(self.xbee.distance[0]*15)
		#self.position.sety(self.xbee.distance[1]*15)

	

def main():
	print("starting main")
	
	#initialize filter
	turtleplot = TurtlePlot()
	#turtleplot.start_turtle()
	#kalmanfilter = calibrate_filter(kalmanfilter) #should calibrate everything (takes aprox 15+ seconds)

	#orientation = kalmanfilter.getOrientation() #pulls the most recent orientation out of the filter, after calibration

	#turtleplot.gui.title("Mapping")
	#turtleplot.gui.geometry("200x50")
	
	#gui = Frame(turtleplot.gui)
	#gui.grid()
	#startButton = Button(gui, text = "Start")
	#startButton.grid()
	#startButton["command"] = turtleplot.start_turtle
	#stopButton = Button(gui, text = "Stop")
	#stopButton.grid()
	#stopButton["command"] = sys.exit()
	#gui.mainloop()
	#start_new_thread(gui.mainloop(), )
	
	#graph initialize
	#print("starting main graph")

	turtleplot.position.setheading(90)
	turtleplot.position.shape("arrow")
	
	
	print("beginning main loop")
	#i=0 #just for the loop 
	while True:
		try:
			time.sleep(1)

			turtleplot.plot_turtle()
		except Exception as e:
			print e
			sys.exit()
	
		
        

if __name__=='__main__':  main()
    

