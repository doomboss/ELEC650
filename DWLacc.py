#Driver for the LSM303D accelerometer and magnetometer/compass

#First follow the procedure to enable I2C on R-Pi.
#1. Add the lines "ic2-bcm2708" and "i2c-dev" to the file /etc/modules
#2. Comment out the line "blacklist ic2-bcm2708" (with a #) in the file /etc/modprobe.d/raspi-blacklist.conf
#3. Install I2C utility (including smbus) with the command "apt-get install python-smbus i2c-tools"
#4. Connect the I2C device and detect it using the command "i2cdetect -y 1".  It should show up as 1D or 1E (here the variable LSM is set to 1D).

#Driver by Fayetteville Free Library Robotics Group
import DWLkf1_0 as kf
import math
import time
import datetime
from smbus import SMBus
busNum = 1
b = SMBus(busNum)
oaccx=0
oaccy=0
oaccz=0
gyoxangle=0
gyoyangle=0
gyozangle=0

G_GAIN = 0.00875

kalman = kf.kalmanFilter()



#acctranslate=2047.97 #value for 16g
acctranslate=16383.8/(4*9.80665) #8g converted to m/s^2
#acctranslate=5461.25 #2g

RAD_TO_DEG = 57.29578 
M_PI = 3.14159265358979323846 # Pi value
DT = 10 #derivative of time, 20ms, using it for the kalman filter

threshold=0.02

LSM = 0x1d #this is the accelerometer and magnetometer sensor
LSM_GYO = 0x6b #this should be the address for the gyro sensor

LSM_WHOAMI_LSM303D = 0b01001001 #Accelerometer and Magnetometer Device self-id
LSM_WHOAMI_L3GD20H = 0b11010111 #This should be the gyroscope?

def twos_comp_combine(msb, lsb): #sensor value: 1 is acceleometer, 2 is gyroscope, 3 is magnetometer
    twos_comp = 256*msb + lsb
    if twos_comp >= 32768:
        return (twos_comp - 65536)
    else:
        return twos_comp

#bittranslation is the value to convert bit to actual value in meter/second
#depends on the g value for accelerometer
def twos_comp_combine_acc(msb, lsb, bittranslation):
    twos_comp = 256*msb + lsb
    if twos_comp >= 32768:
	tmp = (twos_comp - 65536)/bittranslation
        if(math.fabs(tmp) < threshold ):
	    return 0
	else:
	    return tmp
    else:
	tmp = twos_comp/bittranslation
        if( math.fabs(tmp)  < threshold ):
            return 0
        else:
            return tmp

    #tmp = twos_comp/bittranslation
    #if(math.abs(accx)<threshold):
    #    accx=0
    #if(math.abs(accy)<threshold):
    #    accy=0
    #if(math.abs(accz)<threshold):
    #    accz=0

 

#Control register addresses -- from LSM303D datasheet

CTRL_0 = 0x1F #General settings
CTRL_1 = 0x20 #Turns on accelerometer and configures data rate
CTRL_2 = 0x21 #Self test accelerometer, anti-aliasing accel filter
CTRL_3 = 0x22 #Interrupts
CTRL_4 = 0x23 #Interrupts
CTRL_5 = 0x24 #Turns on temperature sensor
CTRL_6 = 0x25 #Magnetic resolution selection, data rate config
CTRL_7 = 0x26 #Turns on magnetometer and adjusts mode

#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
MAG_X_LSB = 0x08 # x
MAG_X_MSB = 0x09
MAG_Y_LSB = 0x0A # y
MAG_Y_MSB = 0x0B
MAG_Z_LSB = 0x0C # z
MAG_Z_MSB = 0x0D

#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
ACC_X_LSB = 0x28 # x
ACC_X_MSB = 0x29
ACC_Y_LSB = 0x2A # y
ACC_Y_MSB = 0x2B
ACC_Z_LSB = 0x2C # z
ACC_Z_MSB = 0x2D

#Registers holding 12-bit right justified, twos-complemented temperature data -- from LSM303D datasheet
TEMP_MSB = 0x05
TEMP_LSB = 0x06

#Registers holding the Gyrosope data from L3GD20H
GYO_X_LSB = 0x28 # x
GYO_X_MSB = 0x29
GYO_Y_LSB = 0x2A # y
GYO_Y_MSB = 0x2B
GYO_Z_LSB = 0x2C # z
GYO_Z_MSB = 0x2D

FIFO_CTRL      = 0x2E # D20H
FIFO_SRC       = 0x2F # D20H

IG_CFG         = 0x30 # D20H
IG_SRC         = 0x31 # D20H
IG_THS_XH      = 0x32 # D20H
IG_THS_XL      = 0x33 # D20H
IG_THS_YH      = 0x34 # D20H
IG_THS_YL      = 0x35 # D20H
IG_THS_ZH      = 0x36 # D20H
IG_THS_ZL      = 0x37 # D20H
IG_DURATION    = 0x38 # D20H

LOW_ODR        = 0x39 # D20H

if b.read_byte_data(LSM, 0x0f) == LSM_WHOAMI_LSM303D:
    print 'LSM303D detected successfully.'
else:
    print 'No LSM303D detected on bus '+str(busNum)+'.'

if b.read_byte_data(LSM, 0x0f) == LSM_WHOAMI_L3GD20H:
    print 'L3GD20H detected successfully...'
else:
    print 'Error detecting L3GD20H on bus ' +str(busNum)+'...   '


    b.write_byte_data(LSM, CTRL_1, 0b01010111) # enable accelerometer, 50 hz sampling
    b.write_byte_data(LSM, CTRL_2, 0b00011000) #set +/- 2g full scale.  0x00 is 2g,0x04 should be 16g?
    b.write_byte_data(LSM, CTRL_5, 0b01100100) #high resolution mode, thermometer off, 6.25hz ODR
    b.write_byte_data(LSM, CTRL_6, 0b01000000) # set +/- 4 gauss full scale
    b.write_byte_data(LSM, CTRL_7, 0b00000000) #get magnetometer out of low power mode
    b.write_byte_data(LSM_GYO, CTRL_1, 0b01101111) # normal mode with XYZ enable
    b.write_byte_data(LSM_GYO, CTRL_4, 0b00000000) 
    b.write_byte_data(LSM_GYO, LOW_ODR, 0b00000000) 
    #b.write_byte_data(LSM_GYO, CTRL_1, 0b00001111) 
    #b.write_byte_data(LSM_GYO, CTRL_1, 0b00001111) 




    #getting the initial value of accleration first
    oaccx = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB), acctranslate)
    oaccy = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_Y_MSB), b.read_byte_data(LSM, ACC_Y_LSB), acctranslate)
    oaccz = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_Z_MSB), b.read_byte_data(LSM, ACC_Z_LSB), acctranslate)
    
    counter=0

    timer = time.time()

while (b.read_byte_data(LSM, 0x0f) == LSM_WHOAMI_LSM303D and b.read_byte_data(LSM_GYO, 0x0f) == LSM_WHOAMI_L3GD20H):
    
    #timer = time.time()
    counter+=1

    while(time.time()-timer < DT/1000):
        time.sleep(100*1/1000000) #sleep 0.1 ms until the total time is 10ms or more
   # delta_t = time.time()-timer
    

    gyox = G_GAIN*twos_comp_combine(b.read_byte_data(LSM_GYO, GYO_X_MSB), b.read_byte_data(LSM_GYO, GYO_X_LSB))
    gyoy = G_GAIN*twos_comp_combine(b.read_byte_data(LSM_GYO, GYO_Y_MSB), b.read_byte_data(LSM_GYO, GYO_Y_LSB))
    gyoz = G_GAIN*twos_comp_combine(b.read_byte_data(LSM_GYO, GYO_Z_MSB), b.read_byte_data(LSM_GYO, GYO_Z_LSB))  
    gyoxangle = DT/1000*gyox
    gyoyangle = DT/1000*gyoy
    gyozangle = DT/1000*gyoz
    #print "Gyroscope (x, y, z):", gyox, gyoy, gyoz

    magx = twos_comp_combine(b.read_byte_data(LSM, MAG_X_MSB), b.read_byte_data(LSM, MAG_X_LSB))
    magy = twos_comp_combine(b.read_byte_data(LSM, MAG_Y_MSB), b.read_byte_data(LSM, MAG_Y_LSB))
    magz = twos_comp_combine(b.read_byte_data(LSM, MAG_Z_MSB), b.read_byte_data(LSM, MAG_Z_LSB))

    #print "Magnetic field (x, y, z):", magx, magy, magz

    accx = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB), acctranslate)
    accy = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_Y_MSB), b.read_byte_data(LSM, ACC_Y_LSB), acctranslate)
    accz = twos_comp_combine_acc(b.read_byte_data(LSM, ACC_Z_MSB), b.read_byte_data(LSM, ACC_Z_LSB), acctranslate)

    #print "Acceleration (x, y, z):", accx, accy, accz
    #print "Distance estimation in meter: ", math.sqrt(math.pow(accx-oaccx,2)+math.pow(accy-oaccy,2))


    #kalman filter from another python script!
    #delta_t = timer - time.time()*1000.0
    #aax = float(kf.AAX(accx,accy,accz)) #x
    #aaz = float(kf.AAZ(accx,accy,accz)) #z
    #aay = float(kf.AAY(accx,accy,accz)) #y
    #KangleX = kalman.X(aax, gyox, delta_t) #gets calculated x
    #KangleY = kalman.Y(aay, gyoy, delta_t) #gets calculated y
    #KangleZ = kalman.Z(aaz, gyoz, delta_t) #gets calculated y
    #print("Kalman angles " +KangleX+","+KangleY+","+KangleX)

    #print(time.time()-timer)
   
    
    
    #delta_t = 1
    aax = float(kf.AAX(accx,accy,accz)) #x
    aaz = float(kf.AAZ(accx,accy,accz)) #z
    aay = float(kf.AAY(accx,accy,accz)) #y
    KangleX = kalman.X(aax, gyox, DT) #gets calculated x
    KangleY = kalman.Y(aay, gyoy, DT) #gets calculated y
    KangleZ = kalman.Z(aaz, gyoz, DT) #gets calculated z
    #print("AAX AAY AAZ ", aax, aay, aaz)
   # print("Kalman angles ", KangleX,",", KangleY,",", KangleZ)

#    temperature = twos_comp_combine(b.read_byte_data(LSM, TEMP_MSB), b.read_byte_data(LSM, TEMP_LSB))
#    print "Temperature Reading: ", temperature

    oaccx = accx
    oaccy = accy
    oaccz = accz

    

    if(counter==20):
	#print "Acceleration (x, y, z):", accx, accy, accz
        #print "Distance estimation in meter: ", math.sqrt(math.pow(accx-oaccx,2)+math.pow(accy-oaccy,2))
	print "Time spent: ", time.time() - timer
	print "Gyroscope(x, y, z):", gyox, gyoy, gyoz

	#print "Magnetic field (x, y, z):", magx, magy, magz

        print("AAX AAY AAZ ", aax, aay, aaz)
	print("Kalman angles ", KangleX,",", KangleY,",", KangleZ)
	counter=0
    #time.sleep(1)
