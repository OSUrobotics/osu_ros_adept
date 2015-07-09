#!/usr/bin/env python
#from __future__ import with_statement # for python 2.5
PKG = 'osu_ros_adept'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import tf_conversions
import PyKDL
from osu_ros_adept.msg import *
from osu_ros_adept.srv import *
from geometry_msgs.msg import *
#from wam_srvs.srv import *
from std_msgs.msg import Int16
from std_srvs.srv import Empty
import math
import random
import os
import pwd
import osu_ros_adept
import openravepy
from openravepy import *
from numpy import *
import scipy
import time
import sys
import itertools
#import FeatureComputer
#from LoadObject import *
#from CoordinateChange import *

class Rave:


	def __init__(self):	

		# This will close OpenRAVE cleanly whenever ROS is closed
		rospy.on_shutdown(self.myshutdown)
		
		#self.camera = 0
		self.axisbody = None
		self.StartPos = 0
		rospy.loginfo("Environment")		
		self.env = Environment()
		rospy.loginfo("End Environment")		
		# Specifies the path to find the environment file, sys.argv[1] is the actual *.env.xml file
		envpath = "/home/roboticslab/ros/catkin_ws/src/osu_ros_adept/scenes/"+ sys.argv[1]
		rospy.logdebug(envpath)	
		self.env.SetViewer('qtcoin')
		self.env.Load(envpath)
		rospy.loginfo("end load path")		
		self.env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)
		self.object = None	
		# Robot and manipulator returned depend on the environment file loaded. 
		self.robot = self.env.GetRobots()[0] # get the first robot	
		self.manip = self.robot.GetManipulators()[0]
		
		#self.robot.SetActiveManipulator('trigrip')
		ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D)
	

		if not ikmodel.load():
			ikmodel.autogenerate()
		
		self.TaskHand = interfaces.TaskManipulation(self.robot)	
		self.BaseManip = interfaces.BaseManipulation(self.robot)
	
        # Subscriber to move the robot based upon commands that are received (UpdateRobotJoints) 
		WAM = rospy.get_param('openrave/WAM', False)
    
		# TODO: fix the subscribe/publish order
		rospy.loginfo("before setting up services")

		# Service requests are actually sent from within the program from UpdateRobotJoints()
		self.serv = rospy.Service('check_pose', PoseCheck, self.handle_pose_check)

		# Following services created and used by robot lab, not currently used by osu_ros_adept
		self.serv2 = rospy.Service('openrave/change_object', ChangeObject, self.change_object)
		self.serv3 = rospy.Service('openrave/grasp_object', GraspObject, self.Grasp_Object)
		self.serv4 = rospy.Service('openrave/record_data', Empty, self.Record_Data)
		self.serv5 = rospy.Service('openrave/change_camera', ChangeObject, self.Change_Camera)


		# Takes joint positions from joy_to_RAVE, UpdateRobotJoints() eventually sends commands 
		# in the form of a service to handle_pose_check()
		self.sub = rospy.Subscriber('joint_commands', RobotPose, self.UpdateRobotJoints, queue_size=1)

		#self.sub2 = rospy.Subscriber('record_data', Int16, self.RecordData)
		#self.sub4 = rospy.Subscriber('wam/pose', PoseStamped, self.MoveTriad)
	
		# Publishes current robot position in the main loop
		self.jointpub = rospy.Publisher('joint_pos', RobotPose)

		# If using the WAM, enable communication to the custom WAM node 
		# which has the WAM services and publishers.
		# This is good for debugging or testing without the WAM connected.
		if WAM != False:
			self.joint_repub = rospy.Publisher('joint_pos', RobotPose)
			self.joint_repub2 = rospy.Publisher('joint_vels', RobotPose)
	
		#wampub = rospy.Publisher('joint_pos', RobotPose)
	
		rospy.loginfo("Done setting up services")




	def myshutdown(self):
		#robot.SetTransform([[0, 0, 1,0],[0,1, 0,0],[-1,0,0,0],[0, 0, 0,1]])
		#camerafile.close()
		RaveDestroy()




	# This is for loading a new object into OpenRAVE
	def change_object(self, num):

		rospy.loginfo("change obj call")
		rospy.logdebug(len(num.transform))
		objectfile, objectname, T ,test = LoadObject(num.data)
		if len(num.transform) == 3:
			try:
				myrot = PyKDL.Rotation.RPY(num.transform[0],num.transform[1],num.transform[2])
				for i in range(3):
					for j in range(3):
						T[i][j] = myrot[i,j]
			except: rospy.logdebug("error creating transform matrix")
		elif len(num.transform) == 6:
			rospy.loginfo("change object call with offset")
			try:
				rospy.logdebug(T)
				T[0][3] = num.transform[3]
				T[1][3] = num.transform[4]
				T[2][3] = num.transform[5]
				rospy.logdebug(T)
				myrot = PyKDL.Rotation.RPY(num.transform[0],num.transform[1],num.transform[2])
				for i in range(3):
					for j in range(3):
						T[i][j] = myrot[i,j]
			except: rospy.loginfo("error creating transform matrix")
		with self.env:
			if self.object is not None:
				self.env.RemoveKinBody(self.object)
				del self.object
			self.object = self.env.ReadKinBodyURI(objectfile)
			if self.object is not None:
				self.object.SetName(objectname)
				myTrimesh = self.env.ReadTrimeshURI(objectfile)
				self.object.InitFromTrimesh(myTrimesh, True)
				self.env.AddKinBody(self.object, True)
				self.object.SetTransform(T)
		return []
 



	# Tries to find the best solution from a list of all solns.
	def find_soln(self, solns, oldJoints):
	# TODO Find a better way to get the best solution, V+ has a NoFlip command that could be part of a fix, velocity based movement could be another choice  
	# Perhaps flatly reject any solutions that have a flip, forcing the user to find another movement

		closest = sys.float_info.max	
		closeIdx = 0	
		# Find a solution with a better joint 1 value
		for i in range(len(solns)):
			tmp = oldJoints[0] - solns[i][0]
			if( tmp < closest):
				closest = tmp
		
		closest2 = sys.float_info.max
		# Find a solution with a better joint 2 value
		for i in range(len(solns)):
			if(solns[i][0] == closest):
				tmp = oldJoints - solns[i][1]				
				if(tmp < closest2):
					closest2 = tmp
					closeIdx = i

		soln = solns[closeIdx]
		return soln



	
	# Send joint commands to controller_comm_client, which will forward them to the Adept
	def send_cmd(self, soln):

		rospy.wait_for_service('send_robot_movements')
		try:
			send_cmd = rospy.ServiceProxy('send_robot_movements', robot_movement_command)
			jts = []
			for i in range(6):
				jts.append(soln[i])

			#Fix offsets between virtual and real robot	
			# OpenRAVE solution appears to be offset from Actual robot positions at joints 1-3
			# Joint 1: Rave1  		= -Adept1
			# Joint 2: Rave2 - pi/2 = Adept2 
			# Joint 3: Rave3 + pi   = Adept3
			# Joint 4: Rave4        = -Adept4 
			# Joint 5: Rave5        = -Adept5		
			jts[0] = - jts[0]
			jts[1] = jts[1] - math.pi/2
			jts[2] = jts[2] + pi
			jts[3] = -jts[3]
			jts[5] = -jts[5]
			
			# Message length 60, message type 1 = joint angles, command type, message reply and message unused = 0 
			params = [60, 1, 0, 0, 0]
			# Actually send the service request
			length = send_cmd(params[0], params[1], params[2], params[3], params[4], jts)
			rospy.logdebug(str(length) + " bytes sent")

		except rospy.ServiceException, err:
			print "Service call failed: %s"%err


# ************ Service to check the given pose to see if it is a valid/reachable location ***********
	# Perform collision check, forward commands to physical robot
	def handle_pose_check(self, msg):
		#TODO: fix the msg input and function calls used in this node
		#self.MoveTriad(msg)
		#self.env.RemoveKinBody(self.object)
	
		#rospy.loginfo("handle_pose function called")
		#rospy.logdebug(msg)

		myrot = PyKDL.Rotation.Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
		r,p,y = PyKDL.Rotation.GetRPY(myrot)
		myrot = PyKDL.Rotation.RPY(r, -p, -y)
		MyFrame = PyKDL.Frame( myrot , PyKDL.Vector(msg.position.x, msg.position.y, msg.position.z) ) 
		FrameMsg1 = tf_conversions.toMsg(MyFrame)
		MyMatrix = tf_conversions.toMatrix(MyFrame)

		# Current location of RAVE model hand
		tmp = self.manip.GetTransform()

		#rospy.logdebug("\n\t\t====Current Matrix====\n" + str(tmp))

		# Convert the message to a matrix, MyMatrix will be used to find an IK solution
		f1 = tf_conversions.fromMsg(msg)
		MyMatrix = tf_conversions.toMatrix(f1)

#		rospy.logdebug("\n\t\t====Matrix For Soln====\n" + str(MyMatrix))
		
		# Spread values unused while working with trigrip
		#if msg.spread or msg.spread == 0:
		#	self.robot.SetDOFValues([msg.spread],[10])

		with self.env:   # Check to see if hand position is in collision.  If so, reset hand position.
			oldJoints = self.robot.GetDOFValues()
			#rospy.logdebug("\n\t====Old joints====\n" + str(oldJoints))
			try:	
				
				IKtype = IkParameterizationType.Transform6D
				#rospy.logdebug(IkParameterization(MyMatrix, IKtype))
				
				# Finds all solutions for a given point and orientation in space
				solns = self.manip.FindIKSolutions(IkParameterization(MyMatrix, IKtype), filteroptions=IkFilterOptions.CheckEnvCollisions)
				#rospy.loginfo("\n\t===solns===\n" + str(solns))

				# Holder for a better solution
				soln = None
				if solns != []:	    
					# Try to find the solution with the least change in joints 1 and 2, tries to minimize jumping
					soln = self.find_soln(solns, oldJoints)
				
				
				# Could be used in lieu of FindIKSolutions, finds just one solution, not necessarily the best
				#soln = self.manip.FindIKSolution(IkParameterization(MyMatrix, IKtype), filteroptions=IkFilterOptions.CheckEnvCollisions)
				
				# We have a solution to apply to the robot
				if(soln != None):
					#rospy.loginfo("\n\t===soln===\n" + str(soln))
					self.robot.SetDOFValues(soln, self.manip.GetArmIndices())
					#Send Command to the physical robot
					if(msg.move):
						# Displays solutions used when A button is pressed
						rospy.loginfo("\n\t===Joint Solution===\n" + str(soln))
						self.send_cmd(soln)

				else:
					rospy.loginfo("No IK solution found")
					#self.env.AddKinBody(self.object)
					return osu_ros_adept.srv.PoseCheckResponse(False,[0,0,0])
			except e:
				rospy.loginfo("IK not available")
				#rospy.logdebug(e)
			report=CollisionReport()

			for link in self.robot.GetLinks():
				linkCollision = self.env.CheckCollision(link,report)
				if linkCollision:
					rospy.logdebug("Link collision " + str(linkCollision) + "Link: " + str(link))

				# Wam code, not used with Adept/Trigrip
				if "wam0" in str(link): # If looking at the base of the WAM, skip and continue with next link
					continue

				incollision=self.env.CheckCollision(link,report)
				if incollision:# and msg.j_position[1] == 0 and msg.j_position[2] == 0 or incollision and msg.j_position[3] != 0:
					rospy.logdebug("Incollision")
					# Detects collision, reset joints
					self.robot.SetDOFValues(oldJoints)
					rospy.loginfo("Collision detected")
					
					#self.env.AddKinBody(self.object)
					return PoseCheckResponse(False,[0,0,0])
		#self.MoveTriad(msg)
		#if not msg.move: # if not moving, reset robot to old Joint values
		#	self.robot.SetDOFValues(oldJoints)
		
		if msg.get_norm:  # Validated: Return norm is useful when using the Interactive markers to control the WAM
			vals = MyMatrix[0:3,2]
			#self.env.AddKinBody(self.object)
			return PoseCheckResponse(True, vals)
		#self.env.AddKinBody(self.object)
		return PoseCheckResponse(True,[0,0,0])
	




# ******** Function to update the robot joints when a message is received **************
	def UpdateRobotJoints(self, msg):
		msg2 = None
		#rospy.loginfo('command received')

		MyTrans = self.manip.GetTransform()   # Get the hand transform
		#ftest = tf_conversions.fromMsg(MyTrans)
		#ftest = tf_conversions.toMatrix(ftest)	
		
		f1 = tf_conversions.fromMsg(msg)
		MyMatrix = tf_conversions.toMatrix(f1)


		# Create message to set robot commands
		move_msg = PoseCheck()
		# Copy Postition/Orientation from RobotPose msg to PoseCheck srv
		move_msg.position = msg.position
		move_msg.orientation = msg.orientation
		move_msg.spread = 0.0

		# Check if A button was pressed, if so initiate robot movement
		if(msg.j_position[0]):
			move_msg.move = True
		else:
			move_msg.move = False	
		# not currently used so set to false by default		
		move_msg.get_norm = False
		
		# PRE OSU_ROS_ADEPT CODE, NOT USED IN ADEPT/TRIGRIP PROGRAM

		# Function to run if using Interactive Markers
		if msg.j_position[3] != 0:
			msg2 = PoseCheck()
			mycog = self.object.GetTransform()
			#Add offset of object and subtract a small amount to fix minor Z offset error.
			MyMatrix[0:3,3] = MyMatrix[0:3,3] + mycog[0:3,3] - MyMatrix[0:3,2]*.0175
			f2 = tf_conversions.fromMatrix(MyMatrix)   # Convert the transform to a ROS frame
			msg2.pose = tf_conversions.toMsg(f2) 
			msg2.move = True
		#rospy.logdebug("before receiving error")
		if msg.j_position[1] == 0 and msg.j_position[2] == 1:
			grasp_msg=GraspObject()
			grasp_msg.grasp_direction="open"
			grasp_msg.spread=False
			grasp_msg.get_joint_vals=False		
			self.Grasp_Object(grasp_msg) # Open the hand
			#self.Grasp_Object(grasp_direction="open") # Open the hand
			
		# END NON OSU_ROS_ADEPT CODE

		if msg2 is not None:  # Move arm to new position
			self.handle_pose_check(msg2)
		else:
			# Sends osu_ros_adept message
			self.handle_pose_check(move_msg)
    
		#TODO: Fix this or remove it for the WAM operation.  Cannot use robot.SetTransform.
		# For Palm grasp option
		if msg.j_position[3] == 2:
			HandDirection = MyMatrix[0:3,2]
			for i in range(0,1000):  # loop until the object is touched or limit is reached
				MyMatrix[0:3,3] += .0001 * HandDirection
				with env:
					self.robot.SetTransform(MyMatrix)  # Move hand forward
				for link in self.robot.GetLinks():    # Check for collisions
					incollision=self.env.CheckCollision(link,report)
					if incollision:
						i = 1000
				if i == 1000:   # If hand is in collision, break from the loop
					break

		# Check to see if any of the manipulator buttons have been pressed.  Otherwise, move the hand.
		if msg.j_position[1] == 1 and msg.j_position[2] == 0:
			grasp_msg=GraspObject()
			grasp_msg.grasp_direction="close"
			grasp_msg.spread=False
			grasp_msg.get_joint_vals=False		
			self.Grasp_Object(grasp_msg) # Open the hand
			#self.Grasp_Object(grasp_direction="close") # Open the hand
		return




 # ************* Function to change the camera angle *********************
	def Change_Camera(self, msg):
		if msg.data == -1:
			self.camera = 1
		elif msg.data == 1:
			self.camera = 0
		return []

		


# ************ Function/Service for setting the hand spread/values and grasping/releasing

	def Grasp_Object(self, msg):
		#TODO: fix the msg input and function calls used in this node
		Vals = zeros(11)
		rospy.logdebug(msg)
		if msg.spread:  # Set the spread of the Barrett Hand
			self.robot.SetDOFValues([msg.spread],[10])
			#rospy.logdebug("this worked")
		if msg.grasp_direction:  # Grasp
			if msg.grasp_direction == "open":
				self.TaskHand.ReleaseFingers()
			elif msg.grasp_direction == "close":
				self.TaskHand.CloseFingers()
			self.robot.WaitForController(0)
			rospy.loginfo("hand done moving")
			if msg.get_joint_vals:  # Get the robot joint values
				Joints = self.robot.GetDOFValues()
				Vals = Joints[0:11] 
			self.robot.GetController().Reset(0)
			rospy.loginfo("return joint values")
			#rospy.logdebug(Vals)
			return GraspObjectResponse(Vals)  # Return joint values
		return GraspObjectResponse([0,0,0])

		

		
		


    # TODO:
    # This is a service call.  Need to remove the subscriber and all of the control
	# programs so that they use the service instead of the publisher/subscriber method.
	def Record_Data(self):
		msg = ChangeObject()
		msg.data = -1
		self.RecordData(msg)

	   


    # ************ Function for recording the joint data and the heuristics ***********
	def RecordData(self, msg):
		rospy.logdebug(msg)
		try:
			#rospy.logdebug('the following message was received: {0}' .format(msg))
			if msg.data == -1:
				# manip.GetTransform gets a different value than robot.GetTransform. 
                # Which to use? Not sure right now.
				# Will continue to use manip in order to stay consistent.		

				myTrans = self.manip.GetTransform()
				myQuaternion = poseFromMatrix(myTrans) # [ qx, qy, qz, qw, tx, ty, tz]

				handpos = myQuaternion[4:7]
				handquat = myQuaternion[0:4]
				HandDirection = myTrans[0:3,0]	
				report=CollisionReport()

				x = 0
				MyContacts = [[0,0,0]]
				# notify checker to return contacts
				for link in self.robot.GetLinks():
					incollision=self.env.CheckCollision(link,report)
					#rospy.logdebug('{0} ; {1}' .format(link, len(report.contacts)) )
					if incollision and len(report.contacts) <=50: 
						link2 = str(link)
						if "-2" in link2: #or "(15)" in link2 or "(17)" in link2:
							if x == 0:
								MyContacts= [report.contacts[0].pos]
								x = 1
							else:
								MyContacts.append(report.contacts[0].pos)
				#rospy.logdebug(MyContacts)       
				#Grippervalues = self.robot.GetDOFValues()  # This is for Barrett hand only
				Grippervalues = myJoints[7:11] # This is for using the WAM 
				mycog = self.object.GetTransform()
				cog = mycog[0:3,3]
				objectcog = poseFromMatrix(mycog)
				# Initialize the feature computer with the correct values and calculate the heuristics
				Features = FeatureComputer.FeatureComputer(env, robot, handpos, handquat, Grippervalues, MyContacts, cog, bodynew, myTrans)
				Results = Features.ComputeAll()
				bodyname = self.object.GetName()
		
				# Write the heuristic data to the file	
				heuristicsfile = open(testpath,'a')
				heuristicsfile.write(', '.join(map(str, Results)) )
				for i in myQuaternion:
					heuristicsfile.write(', ' + str(i) )
				heuristicsfile.write(', ' + bodyname + ', ' )
				heuristicsfile.write(', '.join(map(str, objectcog)) )
				heuristicsfile.write(',' + str(StartPos) + ',' + str(rospy.Time.now()) + '\n')
				heuristicsfile.close()

				self.TaskHand.ReleaseFingers()
				self.robot.WaitForController(0)
				self.robot.GetController().Reset(0)
			# If the change object button was pressed, reset robot and load new object
			else:   
				#with self.env:		
					# Reset the robot and remove the previous object
					#self.robot.SetDOFValues([0,.5,0,.5,0,0,0,0,0,0,0])
					#robot.SetTransform([[0, 0, 1,0],[0,1, 0,0],[-1,0,0,0],[0, 0, 0,1]])
					#self.StartPos = random.randint(1,5)
					#T = getStartLocation(StartPos)
					#robot.SetTransform(T)
				obj = ChangeObject()
				obj.data = msg.data
				rospy.logdebug(obj)
				self.env.Remove(self.object)
				self.change_object(obj)
		except:
			if msg.data == -1:
				rospy.logdebug("an error occurred when trying to get heuristics")
			else:
				rospy.logdebug("an error occurred when trying to change objects")
		



      # ************ Move Axis for debug purposes ************************
	def MoveTriad(self, msg):
		if self.axisbody is None:
			return
		f1 = tf_conversions.fromMsg(msg.pose)
		T = tf_conversions.toMatrix(f1)
		#T[0:3,3] = [-.5,0,0]
		self.axisbody.SetTransform(T)



  
if __name__ == "__main__":
	#RaveInitialize()
	rospy.init_node('OpenRAVE', log_level=rospy.DEBUG)
	ObjectNum = 0
	#modMatrix data is for Barrett hand, not applicable for osu_ros_adept
	modMatrix = PyKDL.Rotation.RPY(0, -3.14159/2, 0)
	modMatrix2 = PyKDL.Rotation.RPY(0, 3.14159/2, 0)
	shake = rospy.get_param('openrave/Shake', False)
    # **************************************************************
    # *************** FUNCTION DEFINITIONS *************************

    # ************ Function call to stop OpenRAVE ******************
    
	MyRave = Rave()
	camera = 0
	obj = ChangeObject()
	obj.data = 0
	#MyRave.change_object(obj)
	# Load OpenRAVE and create the environment using the file declared as 
    # "args" in the launch file
	MyRave.axisbody = MyRave.env.GetKinBody('axis1')

	T = MyRave.manip.GetTransform()
	#MyRave.axisbody.SetTransform(T)
	
	# TODO: Make the file path names be a function of interface and the user ID #
	# TODO: Make writing to the data file be an append function using the user 
    # ID# as another column?
	"""
	# Open the data file and write the header information
	testpath = roslib.packages.get_pkg_dir('ROS_OpenRAVE') + "/testdata.csv"
	#camerapath = roslib.packages.get_pkg_dir('ROS_OpenRAVE') + "/cameradata.csv"
	if not shake:
		#camerafile = open(camerapath,'w')
		#camerafile.write('camera_position, rospy.Time.now(), q1w, q1x, q1y, q1z, q2w, q2x, q2y, q2z\n') 
		heuristicsfile = open(testpath,'w')
		heuristicsfile.write('PointArng, TriangleSize, Extension, Spread, Limit, PerpSym, ParallelSym, OrthoNorm, Volume, Hand_Qx, Hand_Qy, Hand_Qz,Hand_Qw,Hand_x,Hand_y,Hand_z, ObjectName, Object_Qx, Object_Qy, Object_Qz, Object_Qw, Object_x, Object_y, Object_Qz, Start_Position,' + str(rospy.Time.now()) + '\n')
		heuristicsfile.close()
	"""
	# Define the OpenRAVE variables necessary for controlling the robot. See OpenRAVE documentation
	# for more information
    
    
    
	# ****************** Repeat this loop while ROS is running. *************************

	while not rospy.is_shutdown():
		# Get hand position and robot joints to create new message
		MyTrans = MyRave.manip.GetTransform()   # Get the hand transform				
		f2 = tf_conversions.fromMatrix(MyTrans)   # Convert the transform to a ROS frame
		f2 = tf_conversions.toMsg(f2)    #  Convert the frame to a message format for publishing
		joints = MyRave.robot.GetDOFValues()

		
		#rospy.loginfo("Joints\n" + str(joints))
		#rospy.logdebug(MyTrans)

		# Use the values retrieved to create new message of current robot location
		f = RobotPose()	
		f.position = f2.position
		f.orientation = f2.orientation

		#f.j_position = [joints[0],joints[1],joints[2],joints[3]]  # For use when only Barrett hand is loaded
		#f.j_position = [joints[7],joints[8],joints[9],joints[10]] # Hand joints 1-3(7-9) and spread(10) 

		f.j_position = [0.0, 0.0, 0.0, 0.0]
		f.j_velocity = [0.0, 0.0, 0.0, 0.0]  # Currently not used

		# Publish the data and sleep for a short period of time
		MyRave.jointpub.publish(f)
		
		# Efectively sets the rate that joints are published
		rospy.sleep(.1)

rospy.spin()


