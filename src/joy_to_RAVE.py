#!/usr/bin/env python

import roslib; roslib.load_manifest('osu_ros_adept')
import rospy
import time 
import tf
import math
import tf_conversions
#import scipy
from numpy import *

#For hand control, allows us to call sub-processes
import subprocess
#use the command line
from subprocess import call


from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from osu_ros_adept.msg import *
from osu_ros_adept.srv import *
import tf
import PyKDL

#Button indices, DO NOT CHANGE 
A_IDX = 0
B_IDX = 1
X_IDX = 2
Y_IDX = 3
LB_IDX = 4
RB_IDX = 5
BACK_IDX = 6
START_IDX = 7
HOME_IDX = 8
LS_IDX = 9
RS_IDX = 10

#Axes indices, DO NOT CHANGE
LEFT_HOR = 0
LEFT_VERT = 1
LEFT_TRIG = 2
RIGHT_HOR = 3
RIGHT_VERT= 4
RIGHT_TRIG = 5
PAD_HOR = 6
PAD_VERT = 7

speed_msg = 11

#
#
# this code is for using a gamepad to control the open rave model and send commands through rave to the controller.
#
#


# Make a class to encapsulate all of the JoyNode calculations
class joy_to_RAVE:

	   #(x,y,z,r,p,y,fs,fo-c)
    #keyboard = [f/s, e/d, r/v, u/o, i/k, j/l, t/g, y/h]
    #axes = [LJ-X,LJ-Y,LT,RJ-X,RJ-Y,RT,0,0,0,0,0]
    #mybtns:[ 1, 0   , 2, 5,  4, 2,
    buttons = [0,0,0,0,0,0,0,0,0,0,0]
    axes = [0,0,0,0,0,0,0,0,0,0,0]

    def __init__(self):

		self.objectnum = 0	
		
		self.robot_speed = 50
		self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)  #Picks up controller input
		self.sub1 = rospy.Subscriber('joint_pos', RobotPose, self.joints_callback) #Picks up current virtual robot state
		
		self.jointpub = rospy.Publisher('joint_commands', RobotPose)  #Publishes where robot should move

		#setting up the ros packages
		rospy.logdebug('setting up services')
		self.CheckPose = rospy.ServiceProxy("openrave/check_pose", PoseCheck)
		self.Change_Object = rospy.ServiceProxy("openrave/change_object", ChangeObject)
		self.Grasp_Object = rospy.ServiceProxy("openrave/grasp_object", GraspObject)
		self.Change_Camera = rospy.ServiceProxy('openrave/change_camera', ChangeObject)
		
		#Set claw to open
		call(["sudo","/home/roboticslab/ros/catkin_ws/src/osu_ros_adept/maestro_linux/UscCmd","--servo","0,4800"])
		#self.Record_Data = rospy.ServiceProxy('openrave/record_data', Record)
    


    # This callback function will update the global position values
    def joints_callback(self, pos):
		#rospy.logdebug(pos)
		# perform calculations to find new location        
		newpos = tf_conversions.fromMsg(pos)
		newpos = tf_conversions.toMatrix(newpos)

		#rospy.logdebug(newpos)
	
		# Create two matrices to modify the RPY matrix.  This is due to the fact that
		# the BarrettWAM hand is rotated 90deg with respect to the global frame.
		# The M.GetRPY function and Rotation.RPY use the X-axis as the roll axis
		# whereas the WAM wants to roll about the Z-axis.  Must rotate the RPY values
		# 90deg, then calculate the new RPY coordinates, then rotate back to OpenRAVE
		# coordinates again.
		#modMatrix = PyKDL.Rotation.RPY(0, -3.14159/2, 0)
		#modMatrix2 = PyKDL.Rotation.RPY(0, 3.14159/2, 0)

		oldrot = PyKDL.Rotation(newpos[0,0],newpos[0,1],newpos[0,2],newpos[1,0],newpos[1,1],newpos[1,2],newpos[2,0],newpos[2,1],newpos[2,2])
		#oldrot = oldrot * modMatrix  # rotating matrix
	
		# Get the RPY values from the new rotated matrix
		ftest = PyKDL.Frame(oldrot, PyKDL.Vector(0,0,0))
		testrot = ftest.M.GetRPY()

		# Calculate the new positions and roll, pitch, yaw
		roll = testrot[0] + .025 * (self.buttons[RB_IDX] - self.buttons[LB_IDX]) 
		pitch = testrot[1]
		if testrot[1] < -1.5 and self.axes[RIGHT_VERT] > 0 or testrot[1] > 1.5 and self.axes[RIGHT_VERT] < 0:
			pitch = testrot[1]
		else:
			pitch = testrot[1] - .025 * self.axes[RIGHT_VERT]
		yaw = testrot[2] + .025 * self.axes[RIGHT_HOR]
		z = ((self.axes[LEFT_TRIG]-3) - (self.axes[RIGHT_TRIG]-3))

	#
	#
	#  Two different styles of control that move the hand in different ways
	#
	#
		
		
	# Old move controls	
		#newpos[0,3] = newpos[0,3] + .01 * self.axes[LEFT_VERT]
		#newpos[1,3] = newpos[1,3] + .01 * self.axes[LEFT_HOR]
		#newpos[2,3] = newpos[2,3] + .01 * z
		
		#Weight to influence the speed of simulated robot motion				
		weight = .01 

	# "Cockpit" style control method
		newpos[0,3] = newpos[0,3] - weight * self.axes[LEFT_HOR] * math.sin(yaw) + weight * self.axes[1] * math.cos(yaw) * math.cos(pitch) + weight * z * math.sin(pitch) * math.cos(yaw)
		newpos[1,3] = newpos[1,3] + weight * self.axes[LEFT_VERT] * math.sin(yaw) * math.cos(pitch) + weight * self.axes[LEFT_HOR] * math.cos(yaw) + weight * z * math.sin(pitch) * math.sin(yaw)
		newpos[2,3] = newpos[2,3] + weight * z * math.cos(pitch) - weight * self.axes[LEFT_VERT] * math.sin(pitch)
	#rospy.logdebug('cos, sin, yaw: {0} ,{1} ,{2} ' .format(math.cos(yaw), math.sin(yaw), yaw) )


	# Create a frame for publishing.  Make sure to rotate back before creating the frame.
		v = PyKDL.Vector(newpos[0,3],newpos[1,3],newpos[2,3])
		r1 = PyKDL.Rotation.RPY(roll, pitch, yaw)
		#r1 = r1 * modMatrix2
		f1 = PyKDL.Frame(r1, v)	
	
	
	#the hand is non-existent. left over code from the wam. May be useful if someone attached a hand
	# Get the hand values and calculate the new positions
		#spread = pos.j_position[0]
		movement = self.buttons[A_IDX]		
		"""if self.buttons[2] and spread < 3.1:
			spread = spread + .01
		if self.buttons[0] and spread > 0:
			spread = spread - .01
		"""

		
		# publish the new position
		FrameMsg = tf_conversions.toMsg(f1)
		j_pos = array([movement,self.buttons[B_IDX],self.buttons[Y_IDX],0])
		#rospy.logdebug(self.buttons)
		j_vel = array([0,0,0,0])

		MyMsg = RobotPose()
		MyMsg.position = FrameMsg.position
		MyMsg.orientation = FrameMsg.orientation
		MyMsg.j_position = j_pos
		MyMsg.j_velocity = j_vel

		self.jointpub.publish(MyMsg)




    # This callback function will update the global variables
    def joy_callback(self, myjoy):
		self.buttons = myjoy.buttons
		self.axes = myjoy.axes

		#rospy.logdebug(self.axes)
		#rospy.logdebug(self.buttons)

		#rospy.logdebug('the log data button was pressed')
		"""if self.buttons[8] == 1:
			rospy.wait_for_service("openrave/record_data")
			self.Record_Data(1)
		elif self.buttons[7] and self.objectnum < 8:
			self.objectnum = self.objectnum + 1
			rospy.wait_for_service("openrave/change_object",4)
			self.Change_Object(self.objectnum,[])
		elif self.buttons[6] and self.objectnum > 0:
			self.objectnum = self.objectnum - 1
			rospy.wait_for_service("openrave/change_object",4)
			self.Change_Object(self.objectnum,[])
		
		if self.axes[6] != 0:
			rospy.wait_for_service("openrave/change_camera",4)
			self.Change_Camera(self.axes[6],[])
		"""

	# This will call the debug statement in the FeatureComputer.py file
		"""if self.buttons[10] == 1:  #Press in on right analog stick
			rospy.wait_for_service("openrave/record_data",4)
			self.Record_Data(0)
		if self.buttons[9] == 1:  #Press in on right analog stick
			rospy.wait_for_service("openrave/record_data",4)
			self.Record_Data(2)
	"""
		
		
		#X button opens and closes the claw.
		#this goes through the maestro linux package and calls an executable. If you change the hand make sure you know what it's values are for open and close
		if self.axes[PAD_HOR] == 1.0:
			call(["sudo","/home/roboticslab/ros/catkin_ws/src/osu_ros_adept/maestro_linux/UscCmd","--servo","0,3800"])
		elif self.axes[PAD_HOR] == -1.0:
			call(["sudo","/home/roboticslab/ros/catkin_ws/src/osu_ros_adept/maestro_linux/UscCmd","--servo","0,7200"])

			
		#change the robot speed with the up and down on the d-pad
		if self.axes[PAD_VERT] != -0.0:
			if self.axes[PAD_VERT] == 1.0:
				self.robot_speed += 50
				if self.robot_speed > 400:
					self.robot_speed = 400
			else:
				self.robot_speed -= 50
				if self.robot_speed < 50:
					self.robot_speed = 50
			
			rospy.wait_for_service('send_robot_parameters')
			#send over the speed command
			try:
				send_cmd = rospy.ServiceProxy('send_robot_parameters', robot_parameters_command)
				length = send_cmd(20, speed_msg, self.robot_speed, 0, 0)
				print str(length) + " bytes sent"

			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			

		
			

	# D-pad up/down, changes GP grasps if available
		"""if self.axes[7] != 0:
			rospy.wait_for_service("openrave/record_data",4)
			self.Record_Data(self.axes[7])"""
        #rospy.logdebug(self.axes)
	
	# Should not need to publish here since it is done in the other callback function




if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('joy_to_RAVE', log_level=rospy.DEBUG)
    
	
    # Start up a ROSJoyNode
    ROSJoy = joy_to_RAVE()
    rospy.logdebug('joy_to_RAVE started')

    # this loop will cause the joint information to be 
    # published at a rate due to the sleep step size
    # and will calculate the correct amount to move given the 
    # current joint positions and the current joystick state
    """while not rospy.is_shutdown():
        ROSJoy.jointpub.publish(ROSJoy.header, ROSJoy.name, ROSJoy.position, ROSJoy.velocity, ROSJoy.effort)
        rospy.sleep(.10)
	"""
    # Turn control over to ROS
    rospy.spin()
