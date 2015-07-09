#!/usr/bin/env python

import rospy
import sys, time, math, struct
from numpy import *
from osu_ros_adept.srv import *

angle_msg = 1
xyz_msg = 2
speed_msg = 11
accel_msg = 12
tool_msg = 21

num_cmd_params = 5 #Only change if ros.srv.cmd() in adept controller is changed
size = 1024



def send_command(params, joints):
 	
	rospy.wait_for_service('send_robot_movements')
	try:
		send_cmd = rospy.ServiceProxy('send_robot_movements', robot_movement_command)
		temp = []
		for i in range(6):
			temp.append(joints[i])
		
		length = send_cmd(params[0], params[1], params[2], params[3], params[4], temp)
		print str(length) + " bytes sent"

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def send_parameters(params):

	rospy.wait_for_service('send_robot_parameters')
	try:
		send_cmd = rospy.ServiceProxy('send_robot_parameters', robot_parameters_command)
		length = send_cmd(params[0], params[1], params[2], params[3], params[4])
		
		print str(length) + " bytes sent"

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def user_get_xyz(last):

		pos = []
		for i in range(6):
			position = raw_input('XYZ, rpy coord: ')
			if position == "":
				position = last[i]
			elif i == 0:
				if(float(position) > 650 or float(position) < -650):
					print "Joint position is out of range. (-2.96 - 2.96) Setting to 0"
					position = 0
			elif i == 1:
				if(float(position) > 650 or float(position) < -650):
					print "Joint position is out of range. (-3.32 - 0.79) Setting to 0"
					position = 0
			elif i == 2:
				if(float(position) > 900 or float(position) < 25):
					print "Joint position is out of range. (25 - 580) Setting to 0"
					position = 25
			elif i == 3:
				if(float(position) > 180 or float(position) < -180):
					print "Joint position is out of range. (-3.31 - 3.31) Setting to 0"
					position = 0
			elif i == 4:
				if(float(position) > 180 or float(position) < -180):
					print "Joint position is out of range. (-2.08 - 2.2.08) Setting to 0"
					position = 0
			elif i == 5:
				if(float(position) > 180 or float(position) < -180):
					print "Joint position is out of range. (-2*pi - 2*pi) Setting to 0"
					position = 0

			
			pos.append(position)
		last = pos
		
		# Single precision float is needed for the Adept Controller.
		# If wanted, ros.v2 files could be changed to accept doubles instead
		f_angles = array(pos, dtype=float32)

		cmd = []
		cmd.append(60)  #Message Length
		cmd.append(xyz_msg)  #Message Type
		cmd.append(0)  #Message Command
		cmd.append(0)  #Message Reply 
		cmd.append(0)  #Discard bytes
			
		send_command(cmd, f_angles)
		
		return last




def user_get_angles(last):

		pos = []
		for i in range(6):
			position = raw_input('Joint ' + str(i+1) + ': ')
			if position == "":
				position = last[i]
			elif i == 0:
				if(float(position) > 2.96 or float(position) < -2.96):
					print "Joint position is out of range. (-2.96 - 2.96) Setting to 0"
					position = 0
			elif i == 1:
				if(float(position) > 0.78 or float(position) < -3.31):
					print "Joint position is out of range. (-3.32 - 0.79) Setting to 0"
					position = 0
			elif i == 2:
				if(float(position) > 4.46 or float(position) < -0.50):
					print "Joint position is out of range. (-0.50 - 4.46) Setting to 0"
					position = 0
			elif i == 3:
				if(float(position) > 3.31 or float(position) < -3.31):
					print "Joint position is out of range. (-3.31 - 3.31) Setting to 0"
					position = 0
			elif i == 4:
				if(float(position) > 2.08 or float(position) < -2.08):
					print "Joint position is out of range. (-2.08 - 2.2.08) Setting to 0"
					position = 0
			elif i == 5:
				if(float(position) > 2 * math.pi or float(position) < -2 * math.pi):
					print "Joint position is out of range. (-2*pi - 2*pi) Setting to 0"
					position = 0

			
			pos.append(position)
		last = pos
		
		f_angles = array(pos, dtype=float32)

		cmd = []
		cmd.append(60)  #Message Length
		cmd.append(angle_msg)  #Message Type
		cmd.append(0)  #Message Command
		cmd.append(0)  #Message Reply 
		cmd.append(0)  #Discard bytes

		send_command(cmd, f_angles)
		
		return last
	

def set_speed(spd):
	
	cmd = []
	cmd.append(20)  #Message Length
	cmd.append(speed_msg)  #Message Type
	cmd.append(spd)  #Message Command
	cmd.append(0)  #Message Reply 
	cmd.append(0)  #Discard bytes

	send_parameters(cmd)

def set_accel(accel):
	
	cmd = []
	cmd.append(20)  #Message Length
	cmd.append(accel_msg)  #Message Type
	cmd.append(accel)  #Message Command
	cmd.append(0)  #Message Reply 
	cmd.append(0)  #Discard bytes
	
	send_parameters(cmd)

def set_effector(effector):
	
	cmd = []
	cmd.append(20)  #Message Length
	cmd.append(tool_msg)  #Message Type
	cmd.append(effector)  #Message Command
	cmd.append(0)  #Message Reply 
	cmd.append(0)  #Discard bytes

	send_parameters(cmd)

def manual_entry_mode():

	option = ""
	last = [0,0,0,0,0,0]
	last_xyz = [255,0,710,0,0,0]
	while option != "exit":	
		option = raw_input('Enter Command, transformation, Set speed, Set acceleration or set end effector? <c | t | s | a | e>: ')
		if(option == "s"):
			spd = int(raw_input('Enter Speed: '))
			set_speed(spd)
		elif(option == "t"):
			last_xyz = user_get_xyz(last_xyz)
		elif(option == "c"):
			last = user_get_angles(last)
		elif(option == "a"):
			accel = int(raw_input('Enter Acceleration: '))
			set_accel(accel)
		elif(option == "e"):
			effector_length = int(raw_input('Enter End Effector Length: '))
			set_effector(effector_length)


def main():

	rospy.init_node("manual_entry_client")	
  	manual_entry_mode()
	

if __name__ == "__main__":
	main()
