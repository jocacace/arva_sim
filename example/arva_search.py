#!/usr/bin/env python
import rospy
import roslib
import time
import numpy
import math
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from arva_sim.msg import arva
from nav_msgs.msg import Odometry

class SIMPLE_ARVA_SEARCH:
	def __init__(self):
		rospy.init_node("arva_search")

		rospy.Subscriber('mavros/state', State, self.stateCb)
		rospy.Subscriber("/receiver1/signal", arva, self.arvaCb)
		rospy.Subscriber("/mavros/global_position/local", Odometry, self.posCb)
		self.vel_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=0)

		self.artva_x = self.artva_y = self.artva_z = 0
		self.state = State()
		self.arva_ready = False

	def arvaCb(self, msg):
		self.artva_x = msg.arva_signals[0].direction.x
		self.artva_y = msg.arva_signals[0].direction.y
		self.artva_z = msg.arva_signals[0].direction.z
		self.dist = msg.arva_signals[0].distance

		if self.dist > -1:
			self.arva_ready = True
		else:
			self.arva_ready = False

	def stateCb(self, msg):
		self.state = msg	

	def posCb(self, msg):
		self.z = msg.pose.pose.position.z

	def run(self):
		rate = rospy.Rate(10.0)	
		max_vel = 1.0
		sp = PositionTarget()
		sp.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 512 + 1024
		sp.coordinate_frame = 8
		ks = 0.5 		#% vertical gain
		kv = 0.1 		#% speed gain
		vmax = 1.0 	#% [m/s] maximum speed
		rs = 1 			#% reference height


		rospy.wait_for_service('mavros/cmd/arming')
		armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
		takeoffService(yaw = 1.5, latitude = 44.4928127, longitude = 11.3299824, altitude = 2.5)
		
		time.sleep(2)
	
		flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
		flightModeService(custom_mode='OFFBOARD')
		while not rospy.is_shutdown():
			if self.arva_ready:

				artva_data = numpy.array([self.artva_x, self.artva_y, self.artva_z])
				vers = artva_data / numpy.linalg.norm(artva_data)
				d =  self.dist / 100.0
				k = vmax*(kv*d)/math.sqrt(1+(kv*d)**2)
				vxy = numpy.array( [k*vers[0], k*vers[1]] )
				vz = ks*(rs-self.z);

				sp.velocity.x = -vxy[1]
				sp.velocity.y = vxy[0]
				sp.velocity.z = vz
				sp.yaw_rate = 0.0

			else:
				sp.velocity.x = sp.velocity.y = sp.velocity.z = sp.yaw_rate = 0.0
			
			self.vel_pub.publish( sp  );
			rate.sleep()

if __name__ == '__main__':
	arva_client = SIMPLE_ARVA_SEARCH()
	arva_client.run()

