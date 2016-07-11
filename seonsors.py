# Simple demo of reading the difference between channel 1 and 0 on an ADS1x15 ADC.
# Author: Tony DiCola
# License: Public Domain
import time
import numpy as np

# Import the ADS1x15 module.
import Adafruit_ADS1x15


# Create an ADS1115 ADC (16-bit) instance.
#adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
adc1 = Adafruit_ADS1x15.ADS1015(address=0x48) #static pressure and kinematic pressure
adc2 = Adafruit_ADS1x15.ADS1015(address=0x49) #Exit pressure and Load cell

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.

#Order of values in 'value' [time, static pressure, knematic pressure, exit pressure, thrust]
value = np.array([0,0,0,0,0])

#start time
start=time.time()

print('Press \'Ctrl-C\' to when done')

#pressing 'Ctrl-C' will exit the loop without exiting the program
try:
	while True:
		# Read the difference between channel 0 and 1 (i.e. channel 0 minus channel 1).
		# Note you can change the differential value to the following:
		#  - 0 = Channel 0 minus channel 1 (static and exit pressures)
		#  - 1 = Channel 0 minus channel 3
		#  - 2 = Channel 1 minus channel 3
		#  - 3 = Channel 2 minus channel 3 (kinematic pressure and Load cell)
		current = time.time() - start
		staticP = adc1.read_adc_difference(0, gain=2/3)*(6.144/(2**11-1))
		kinematicP = adc1.read_adc_difference(3, gain=2/3)*(6.144/(2**11-1))
		exitP = adc2.read_adc_difference(0, gain=2/3)*(6.144/(2**11-1))
		load = adc2.read_adc_difference(3, gain=2/3)*(1.024/(2**11-1))

		#This line for testing
		print(current, staticP, kinematicP, exitP, load)

                value = value.append([current, staticP, kinematicP, exitP, load])
    		
except KeyboardInterrupt:
		pass

np.savetxt('volts.csv', value, '%1.6f', ',', '\n', 'Time (s), Static Pressure, Kinematic Pressure, Exit Pressure, Thrust'
