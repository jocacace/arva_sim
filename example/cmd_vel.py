#!/usr/bin/env python


import rospy
import roslib
import tf
import math
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import time

# Flight modes class
# Flight modes are activated using ROS services
class velocity_controller:
	def __init__(self):

		# Drone state
		self.state = State()
		# Instantiate a setpoints message
		self.sp = PositionTarget()

		'''
		uint16 IGNORE_PX = 1 # Position ignore flags
		uint16 IGNORE_PY = 2
		uint16 IGNORE_PZ = 4
		uint16 IGNORE_VX = 8 # Velocity vector ignore flags
		uint16 IGNORE_VY = 16
		uint16 IGNORE_VZ = 32
		uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
		uint16 IGNORE_AFY = 128
		uint16 IGNORE_AFZ = 256
		uint16 FORCE = 512 # Force in af vector flag
		uint16 IGNORE_YAW = 1024
		uint16 IGNORE_YAW_RATE = 2048
		'''	
		self.sp.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 512 + 1024

		# LOCAL_NED
		self.sp.coordinate_frame = 8
		self.local_pos = Point(0.0, 0.0, 0.0)
		rospy.Subscriber('mavros/state', State, self.stateCb)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.posCb)
		rospy.Subscriber('cmd_vel', Twist, self.velCb)
		self.vel_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=0)
		self.cmd_v = Twist()

	
	def setTakeoff(self):
		rospy.wait_for_service('mavros/cmd/takeoff')
		try:
			takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = 3)
		except rospy.ServiceException, e:
			print "Service takeoff call failed: %s"%e

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setDisarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException, e:
			print "Service disarming call failed: %s"%e

	def setStabilizedMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='STABILIZED')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Offboard Mode could not be set."%e

	def setAltitudeMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='ALTCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Altitude Mode could not be set."%e

	def setPositionMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='POSCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Position Mode could not be set."%e

	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Autoland Mode could not be set."%e


	def velCb(self, msg):
		self.cmd_v = msg

	def posCb(self, msg):
		self.local_pos.x = msg.pose.position.x
		self.local_pos.y = msg.pose.position.y
		self.local_pos.z = msg.pose.position.z

	def stateCb(self, msg):
		self.state = msg

	def run(self):
		rate = rospy.Rate(10.0)

		# Make sure the drone is armed
		while not self.state.armed:
			self.setArm()
			rate.sleep()

		self.setTakeoff()
		time.sleep(2)
		self.setOffboardMode()

		while not rospy.is_shutdown():
			self.sp.velocity.x = -self.cmd_v.linear.y
			self.sp.velocity.y = self.cmd_v.linear.x
			self.sp.velocity.z = self.cmd_v.linear.z
			self.sp.yaw_rate = self.cmd_v.angular.z
			self.vel_pub.publish( self.sp  );
			rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('setpoint_node', anonymous=True)
		vc = velocity_controller()
		vc.run();
	except rospy.ROSInterruptException:
		pass

