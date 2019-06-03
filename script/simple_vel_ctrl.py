#!/usr/bin/env python

import rospy
import roslib
import tf
import numpy
import math  
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
#from artva_gazebo_plugin.msg import artva
from arva_sim.msg import arva
from nav_msgs.msg import Odometry

class artva_vel_ctrl():
	def __init__(self):
		rospy.init_node("artva_vel_ctrl")
		nodename = rospy.get_name()
		rospy.loginfo("%s started" % nodename)
		#self.rate = rospy.get_param("~rate", 10)
		#input_topic = rospy.get_param("~input_topic", "/joy")
		#output_topic = rospy.get_param("~output_topic", "/cmd_vel")

		rospy.Subscriber("arva", arva, self.artvaCb)
		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0) 
		rospy.Subscriber("/mavros/global_position/local", Odometry, self.posCb)

		self.artva_x = 0
		self.artva_y = 0
		self.artva_z = 0

		#self.vx = 0
		#self.vy = 0
		#self.vz = 0
		#self.vyaw = 0
		self.dist = 0
		self.z = 0.0
		self.arva_ready = False

	def artvaCb(self, msg):
		#temp: considero solo 1 artva transmitter1
		self.artva_x = msg.arva_signals[0].direction.x
		self.artva_y = msg.arva_signals[0].direction.y
		self.artva_z = msg.arva_signals[0].direction.z
		self.dist = msg.arva_signals[0].distance
		if self.dist > -1:
			self.arva_ready = True
		else:
			self.arva_ready = False
		#print self.artva_x, " ", self.artva_y, " ", self.artva_z, "\n"


	def posCb(self, msg):
		self.z = msg.pose.pose.position.z

	def spin(self):

		r = rospy.Rate(10)
		max_vel = 1.0

		vel_z = 0.0
		c_vel = Twist()
		###### main loop  ######

		norm = 200 
		while not rospy.is_shutdown():
			#c_vel.linear.x = self.vx
			#c_vel.linear.y = self.vy
			#c_vel.linear.z = self.vz
			#c_vel.angular.z = self.vyaw
			#self.vel_pub.publish( c_vel )
			
			
			#ks = 0.1; % vertical gain
			#kv = 0.1; % speed gain
			#vmax = 0.5; % [m/s] maximum speed
			#
			#rs = 1; % reference height
			#
			#d = (norm(HI))^(-1/3);
			#vers = HI/norm(HI);
			#vxy = vmax*(kv*d)/sqrt(1+(kv*d)^2)*vers(1:2);
			#
			#vz = ks*(rs-s);
			#
			#u = [vxy;vz];
			


			if self.arva_ready:

				ks = 0.5 	#% vertical gain
				kv = 0.1 		#% speed gain
				vmax = 1.0 	#% [m/s] maximum speed
				rs = 1 			#% reference height

								
				artva_data = numpy.array([self.artva_x, self.artva_y, self.artva_z])
				vers = artva_data / numpy.linalg.norm(artva_data)
				d =  self.dist / 100.0
				k = vmax*(kv*d)/math.sqrt(1+(kv*d)**2)
				print "k: ", k, " d: ", d
				vxy = numpy.array( [k*vers[0], k*vers[1]] )
				vz = ks*(rs-self.z);
				print "Vers: ", vers[0], " ", vers[1]

				print "Z_e: ", (rs-self.z)

				c_vel.linear.x = vxy[0]
				c_vel.linear.y = vxy[1]
				c_vel.linear.z = vz

				'''
				#
				#vxy = vmax*(kv*d)/sqrt(1+(kv*d)^2)*vers(1:2);
				#vz = ks*(rs-s);
		

				c_vel.linear.x = artva_data[0]
				c_vel.linear.y = artva_data[1]
				c_vel.linear.z = artva_data[2]

				if self.z < 0.1:
					vel_z = 0.1
				else:
					vel_z = artva_data[2]
				sa = 1.0;
			
				#vel_z = 0.0
				n_vel = numpy.linalg.norm([c_vel.linear.x, c_vel.linear.y, c_vel.linear.z])
				if n_vel > max_vel:
					sa = max_vel/n_vel
        
				cmd_vel = numpy.array([c_vel.linear.x, c_vel.linear.y, c_vel.linear.z])
				cmd_vel = cmd_vel * sa

				c_vel.linear.x = cmd_vel[0]
				c_vel.linear.y = cmd_vel[1]
				c_vel.linear.z = vel_z #cmd_vel[2]

				print "Cmd vel: ", c_vel.linear.x, " ", c_vel.linear.y, " ", c_vel.linear.z
				print "norm: ", numpy.linalg.norm([c_vel.linear.x, c_vel.linear.y, c_vel.linear.z])
				print "self.dist: ", self.dist 
				#norm = numpy.linalg.norm([c_vel.linear.x, c_vel.linear.y, c_vel.linear.z])
				#c_vel.linear.x = c_vel.linear.y = c_vel.linear.z = 0.0;
				'''
			else:
				c_vel.linear.x = 0.0
				c_vel.linear.y = 0.0
				c_vel.linear.z = 0.0

			self.vel_pub.publish( c_vel )
			#print "vel: ", c_vel

			r.sleep()

if __name__ == '__main__':
	""" main """
	try:
		artvavel = artva_vel_ctrl()
		artvavel.spin()
	except rospy.ROSInterruptException:
		pass
