#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
The MIT License (MIT)

Copyright (c) 2016 Nishit Srivastava <nishiitsri@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
## get the relevant libraries to allow raspberry pi to connect with ADC and open a serial port

from Adafruit_ADS1x15 import ADS1x15
import time
import math
import serial

ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)   ## This opens the serial port to be opened with baudrate =57600 and wait for 1 sec before returning all the bytes
ser.setXonXoff(True)  ## enables flow control on the serial port
pga = 6144 ## sets the Programmable Gain Amplifier (PGA) value for the ADS1115 ADC to the full-scale range of ±6.144V.
ADS1115 = 0x01 ##model of ADC used
adc = ADS1x15(ic=ADS1115)
sps = 8 ##samples per second

def read_stress():
	return(float(input("Enter the stress to be applied on the gel")))   ## User input for the stress to be applied on the gel


def voltage_measurement():
	
	voltage_positive = adc.readADCSingleEnded(1, pga, sps)/1.01  #reads voltage at channel 1 and applies an offset
	voltage_negative = -adc.readADCSingleEnded(3, pga, sps) #reads voltage at channel 3 and an offset
	if (abs (voltage_positive) > abs (voltage_negative)):
		currentvoltage = voltage_positive
				
	else:
		currentvoltage = voltage_negative
	
	return currentvoltage
	
def conversion(stress):
	#define conversion() as voltage= function of 
	#stress = read_stress()
	force =  0.00006 * stress
	voltage= force*20/.0098
	#return force
	return voltage

def getStatus():
	ser.write("1TS\r\n")
	d = ser.readline()
	return d
	

def conversion_voltagetostress(voltage) :
	#converts voltage to stress#
	stress = (voltage * 0.0098/20)/0.00006
	return stress
	

def go_up():  ## actuator to go up
	ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
	ser.setXonXoff(True)
	ser.write("1va0.4\r\n")
	ser.write("1PR-0.5\r\n")
	
	
def feedback_voltage(stress):
        ser.write("1va0.04\r\n") #reduce the speed#
        ser.write("1ac0.4\r\n") #reduce the acceleration#
	currentvoltage = voltage_measurement()
	targetvoltage = conversion(stress)
	threshold = 2 #the threshold (in mV) to 2 voltages being defined as equal#
        #threshold value corresponds to 16.333N/m2#
	previousvoltage = 0
	#ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
	step_size = 0.02 #distance travelled by actuator in mm#

	# Positioning the plunger

	while abs(currentvoltage-targetvoltage)>threshold:
	
		if currentvoltage<targetvoltage:
			#print("moving the actuator to decrease load")
			if previousvoltage>targetvoltage:
				print("moving the actuator to decrease load")
				step_size=step_size/2.
				#step_size=0.1*math.floor(step_size*10.)
				if step_size<0.0002: #min. incremental motion is 0.2um#
					step_size = step_size*2
			ser.write("1PR-%.6f\r\n"%step_size)
		else:
        
			if previousvoltage<targetvoltage:
				print("moving the actuator to increase load")
				step_size=step_size/2.
				#step_size=0.1*math.floor(step_size*10.)
				if step_size<0.0002: 
					step_size = step_size*2
			ser.write("1PR%.6f\r\n"%step_size)
			
		previousvoltage=currentvoltage
		currentvoltage= voltage_measurement()
		stress_final = conversion_voltagetostress(targetvoltage)
		stress_current = conversion_voltagetostress(currentvoltage)
		output_stress = 'The value of final stress is ' + repr(stress_final) + ' N/m2, and current stress is ' + repr(stress_current) + 'N/m2, and step size is' + repr(step_size)
		fa = open("/home/nishit/Adafruit-Raspberry-Pi-Python-Code/Adafruit_ADS1x15/data/squasher_dicty_movies/27.6.14/1%agarose/test_1.csv",'a') #opens file which saves the log of device according to the date#
		fa.write(str(stress_final)+','+str(stress_current)+','+str(step_size)+'\n')
		fa.close()
		print output_stress
		#print targetvoltage, currentvoltage, step_size

	while True:
		# check for changes in stress#
		while abs(currentvoltage-targetvoltage)<threshold:
			time.sleep(0.25)#time delay after which load is checked#
			currentvoltage=voltage_measurement()
			stress_current = conversion_voltagetostress(currentvoltage)
			output_stress_interval = 'After 0.25 second,the value of final stress is ' + repr(stress_final) + ' N/m2 and current stress is ' + repr(stress_current) + ' N/m2'
			fa = open("/home/nishit/Adafruit-Raspberry-Pi-Python-Code/Adafruit_ADS1x15/data/squasher_dicty_movies/27.6.14/1%agarose/test_1.txt",'a') #opens file which saves the log of device according to the date#
			fa.write(str(stress_final)+','+str(stress_current)+'\n')
			fa.close()
			print output_stress_interval
		# correct position when required#
		step_size = 0.0005 #move the actuator by 0.5um#
		while abs(currentvoltage-targetvoltage)>threshold:
		
			if currentvoltage<targetvoltage:
				print("moving the actuator to decrease load")
				if previousvoltage>targetvoltage:
					print("moving the actuator to decrease load")
					step_size=step_size/2.
					#step_size=0.1*math.floor(step_size*10.)
					if step_size<0.0002: 
						step_size= step_size*2
				ser.write("1PR-%.6f\r\n"%step_size)
			else:
	        
				if previousvoltage<targetvoltage:
					print("moving the actuator to increase load")
					step_size=step_size/2.
					#step_size=0.1*math.floor(step_size*10.)
					if step_size<0.0002: 
						step_size = step_size*2
				ser.write("1PR%.6f\r\n"%step_size)
				
			previousvoltage=currentvoltage
			currentvoltage= voltage_measurement()
			stress_final = conversion_voltagetostress(targetvoltage)
			stress_current = conversion_voltagetostress(currentvoltage)
			output_stress_readjust = 'The value of final stress is ' + repr(stress_final) + ' N/m2, and readjustedcurrent stress is ' + repr(stress_current) + ' N/m2 and step size is' + repr(step_size)
			fa = open("/home/nishit/Adafruit-Raspberry-Pi-Python-Code/Adafruit_ADS1x15/data/squasher_dicty_movies/27.6.14/1%agarose/test_1.csv",'a') #opens file which saves the log of device according to the date and time#
			fa.write(str(stress_final)+','+str(stress_current)+','+str(step_size)+'\n')
			fa.close()
			print output_stress_readjust
	



      
		   

	

