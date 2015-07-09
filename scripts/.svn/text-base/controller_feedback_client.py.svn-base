#!/usr/bin/env python

import rospy
import socket, struct, sys, math
from osu_ros_adept.msg import *

host = '172.16.120.147' #default controller IP address
port = 11002 #controller feedback server port
size = 1024


def connect():

	s = None	
	try: 
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((host,port))
	except socket.error, (value,message):
		if s:
			s.close()
		print "Could not open socket: " + message
		sys.exit(1)
	return s


def parse_data(d):

	length = struct.unpack("<i",d[0:4])[0]
	msg_type = struct.unpack("<i",d[4:8])[0]
	cmd_type = struct.unpack("<i",d[8:12])[0]
	msg_reply = struct.unpack("<i",d[12:16])[0]
	msg_unused = struct.unpack("<i",d[16:20])[0]
	joints = []


	for i in range(6):
		joints.append(struct.unpack("<f", d[(20+4*i):(24+4*i)])[0])
	print joints
	
	"""print "length = " + str(length)
	print "msg type " + str(msg_type)
	print "cmd type " + str(cmd_type)
	print "msg reply " + str(msg_reply)
	print "msg unused " + str(msg_unused)
	
	for i in range(6):
		print "joint " + str(i+1) + " = " + str(joints[i])"""
	
	return joints

def rec_data(s):
	
	jts_pub = rospy.Publisher("joints", joint_positions)
	while not rospy.is_shutdown():
		data = str(s.recv(size))
		if not data:
			continue
		jts = parse_data(data)	
		print jts	
		rospy.loginfo(jts)
		jts_pub.publish(jts)

def main():

	rospy.init_node("controller_feedback")	
	sock = connect()
	rec_data(sock)
	

if __name__ == "__main__":
	main()
