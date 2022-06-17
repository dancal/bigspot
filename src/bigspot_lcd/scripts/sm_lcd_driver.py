
import I2C_LCD_driver
import datetime
import time
import os
import psutil
import rospy
from math import pi
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from netifaces import interfaces, ifaddresses, AF_INET
from rpi_bad_power import new_under_voltage

class SpotMicroLcd():
	''' Class to encapsulate lcd driver for spot micro robot '''

	def __init__(self):
		'''Constructor'''
		self._mylcd = I2C_LCD_driver.lcd()

		self._empty				= '%'
		self._empty				= self._empty.ljust(16,' ')

		self._spot_state_str	= 'ready'
		self._speed				= '1'
		self._under_voltage		= 'Under voltage'
		self._under_voltage		= self._under_voltage.ljust(16,' ')

		rospy.init_node('lcd_monitor_node', anonymous=True)

		#rospy.Subscriber('sm_speed_cmd',Vector3,self.update_speed_cmd)
		#rospy.Subscriber('sm_angle_cmd',Vector3,self.update_angle_cmd)
		rospy.Subscriber('notspot_lcd/state', String, self.update_state_string)
		rospy.Subscriber('notspot_lcd/joy_speed', String, self.update_joy_speed_string)
		rospy.loginfo(f"LCD init")

	def getLocalIps(self, ifaceName):
		return [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'-.-.-.-'}] )][0]

	def temperature_of_raspberry_pi(self):
		cpu_temp = os.popen("vcgencmd measure_temp").readline()
		return cpu_temp.replace("temp=", "").replace('.', 'C')
 
	def cpu_use_percent(self):
		#l1, l2, l3 = psutil.getloadavg()
		#CPU_use = (l3/os.cpu_count()) * 100
		return psutil.cpu_percent(interval=0.5)

	def update_state_string(self, msg):
		''' Updates angle command attributes'''
		self._spot_state_str 		= msg.data

	def update_joy_speed_string(self, msg):
		''' Updates angle command attributes'''
		self._speed 			= msg.data
	
	def run(self):
		''' Runs the lcd driver and prints data'''

		# Define the loop rate in Hz
		under_voltage 	= new_under_voltage()
		loopIdx			= 0
		#rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			self._wlan0_str			= self.getLocalIps('wlan0')
			self._wlan1_str			= self.getLocalIps('wlan1')

			self._temperature		= self.temperature_of_raspberry_pi()
			self._cpu_percent		= str(int(round(self.cpu_use_percent(),0)))
			self._ram_percent		= str(int(round(psutil.virtual_memory().percent,0)))
			self._speed_str			= str(int(round(float(self._speed),0)))
			
			self._wlan0_str			= self._wlan0_str.ljust(16,' ')
			self._wlan1_str			= self._wlan1_str.ljust(16,' ')
			self._speed_str			= self._speed_str.ljust(16,'X')
			self._temperature		= self._temperature.ljust(16,' ')
			
			if under_voltage.get():
				self._mylcd.lcd_display_string('E:%sself._under_voltage'%(self._under_voltage),1)
			else:
				self._mylcd.lcd_display_string('M:%5s speed:%s'%(self._spot_state_str, self._speed_str),1)
				if (loopIdx == 0) or (loopIdx == 1):
					self._mylcd.lcd_display_string('F%s'%(self._wlan0_str, ),2)
				elif (loopIdx == 2) or (loopIdx == 3):
					self._mylcd.lcd_display_string('S%s'%(self._wlan1_str),2)
				else:
					self._mylcd.lcd_display_string('C:%2s%s R:%2s T:%s'%(self._cpu_percent, '%', self._ram_percent, self._temperature),2)

			# Sleep till next loop
			loopIdx += 1
			if ( loopIdx > 10 ):
				loopIdx	= 0

			d = rospy.Duration(1, 0)
			rospy.sleep(d)
			#rate.sleep(1)
			#time.sleep(0.1)

		self._mylcd.lcd_display_string('LCD Shutdown....',1)
		rospy.loginfo(f"LCD Shutdown")

def main():
	sm_lcd_obj = SpotMicroLcd()
	sm_lcd_obj.run()
