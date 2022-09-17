#!/usr/bin/env python3
import rospy
# Library that contains standard and geometry ROS msgs

import math
from geometry_msgs.msg import Twist

class open_loop_ctrl:
	def __init__(self):
		#Set the time to move the robot in a straight line
		self.time=10
		# Setup Variables to be used
		self.first = True
		self.start_time=0.0
		self.current_time=0.0
		
		self.goals = [[10,10], [-15,-8], [17,-14]]
		self.angulo_inicial = 0
		self.posicion = [0,0]
		self.index = 0


		#Setup Control Message to be used
		self.control = Twist()
		self.control.linear.x = 0.0
		self.control.linear.y = 0.0
		self.control.linear.z = 0.0
		self.control.angular.x = 0.0
		self.control.angular.y = 0.0
		self.control.angular.z = 0.0

		#Setup ROS publishers
		self.vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    	
	def run(self):
		#Run this code only at the beginning
		if (self.first):
			#Wait for the first time to be published from gazebo
			while not rospy.get_time():
				pass
			#Initialise variables to use the first iteration before entering the loop
			self.current_time=rospy.get_time()
			self.start_time=rospy.get_time()
			self.first=False
			
			self.index = 0
			self.angulo_inicial = 0
			self.posicion = [0,0]
            
		else:
			#Calculate time
			self.current_time=rospy.get_time()
			dt=self.current_time-self.start_time
			#If the time is greater than the established time to go straight stop the robot
			
			if (self.index >= 3):
				self.stop()

			else:
				#Set the desired linear and angular speeds of the robot
				#self.control.linear.x= 0.8
				#self.control.angular.z = 0.0
	
				#Publish the control inputs
				#self.vel_pub.publish(self.control)
				
				self.angulo()
				self.llegar()
				self.index = self.index + 1
				
	
	def angulo(self):
		meta = [self.posicion[0] - self.goals[self.index][0], self.posicion[1] - self.goals[self.index][1]]
        	
		if (meta[0] >= 0 and meta [1] >= 0):
			angulo_meta = abs(math.degrees(math.atan(meta[1]/meta[0])))
			angulo_actual = angulo_meta - self.angulo_inicial
		elif (meta[0] <= 0 and meta [1] >= 0): 
			angulo_meta = 180 - abs(math.degrees(math.atan(meta[1]/meta[0])))
			angulo_actual = angulo_meta - self.angulo_inicial
		elif (meta[0] <= 0 and meta [1] <= 0 ):
			angulo_meta = -180 + abs(math.degrees(math.atan(meta[1]/meta[0])))
			angulo_actual = angulo_meta - self.angulo_inicial
		elif (meta[0] >= 0 and meta [1] <= 0):
			angulo_meta = - abs(math.degrees(math.atan(meta[1]/meta[0])))
			angulo_actual = angulo_meta - self.angulo_inicial
		self.angulo_inicial = angulo_meta
		if (angulo_meta - self.angulo_inicial >= 180):
			angulo_actual = -(360 - angulo_meta - self.angulo_inicial)
		elif (angulo_meta - self.angulo_inicial <= -180):
			angulo_actual = 360 + angulo_meta - self.angulo_inicial
        		
		angulo_actual = angulo_meta - self.angulo_inicial 

        	
		self.current_time = rospy.get_time()
		self.start_time = rospy.get_time()
		
		
        	
		while (True):
			self.current_time=rospy.get_time()
			dt=self.current_time-self.start_time
			self.vel_pub.publish(self.control)
        		
			if (angulo_actual > 10):
				self.control.angular.z = angulo_actual * 1
				angulo_actual = angulo_actual - self.control.angular.z * dt
			elif (angulo_actual < -10):
				self.control.angular.z = angulo_actual * 1
				angulo_actual = angulo_actual - self.control.angular.z * dt
				
			elif (angulo_actual > 1): 
				self.control.angular.z = angulo_actual * .7
				angulo_actual = angulo_actual - self.control.angular.z * dt
			elif (angulo_actual < -1): 
				self.control.angular.z = angulo_actual * .7				
				angulo_actual = angulo_actual - self.control.angular.z * dt
			else:
				self.control.angular.z = 0
				self.vel_pub.publish(self.control)
				break
			
	
	def llegar (self):
		meta = [self.posicion[0] - self.goals[self.index][0], self.posicion[1] - self.goals[self.index][1]]
		distancia = math.sqrt(pow(meta[0], 2) + pow(meta[1], 2))
		self.current_time=rospy.get_time()
		self.start_time=rospy.get_time()
		self.position = [self.goals[self.index][0], self.goals[self.index][1]]

        	
		while (True):
			self.current_time=rospy.get_time()
			dt = self.current_time - self.start_time
			self.vel_pub.publish(self.control)
			if (distancia > 1):
				self.control.linear.x =  distancia * 0.4
				distancia = distancia - self.control.linear.x * dt
			elif (distancia >.2):
				self.control.linear.x =  distancia * 0.5
				distancia = distancia - self.control.linear.x * dt
			else:
				self.control.linear.x =  0
				self.vel_pub.publish(self.control)
				break
  			
  			        	
	
    #Stop Condition
	def stop(self):
		#Setup the stop message (can be the same as the control message)
		print("Stopping")
		msg = Twist()
		msg.linear.x = 0
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0
		self.vel_pub.publish(msg)

if __name__ == '__main__':
    #Initialise and Setup node
	rospy.init_node("Move_Straight")
	RobotCtrl = open_loop_ctrl()
	loop_rate = rospy.Rate(10)
	rospy.on_shutdown(RobotCtrl.stop)

    #Run node
	print("Run")    
	try:
		while not rospy.is_shutdown():
			RobotCtrl.run()
			loop_rate.sleep()

	except rospy.ROSInterruptException:
		pass
