#! /usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Float64

class MoveMannequin(object):
	def __init__(self):
		# define all of the publishers
		
		#arms
		self.left_shoulder_roll = rospy.Publisher("/mannequin/left_shoulder_roll/command", Float64, queue_size=1)
		self.left_shoulder_pitch = rospy.Publisher("/mannequin/left_shoulder_pitch/command", Float64, queue_size=1)
		#self.left_shoulder_yaw = rospy.Publisher("/mannequin/left_shoulder_yaw/command", Float64, queue_size=1)
		self.left_elbow_flexion = rospy.Publisher("/mannequin/left_elbow_flexion/command", Float64, queue_size=1)
		self.left_elbow_pronation = rospy.Publisher("/mannequin/left_elbow_pronation/command", Float64, queue_size=1)
		self.left_wrist_flexion = rospy.Publisher("/mannequin/left_wrist_flexion/command", Float64, queue_size=1)
		self.left_wrist_deviation = rospy.Publisher("/mannequin/left_wrist_deviation/command", Float64, queue_size=1)
		
		self.right_shoulder_pitch = rospy.Publisher("/mannequin/right_shoulder_pitch/command", Float64, queue_size=1)
		self.right_shoulder_roll = rospy.Publisher("/mannequin/right_shoulder_roll/command", Float64, queue_size=1)
		#self.right_shoulder_yaw = rospy.Publisher("/mannequin/right_shoulder_yaw/command", Float64, queue_size=1)
		self.right_elbow_flexion = rospy.Publisher("/mannequin/right_elbow_flexion/command", Float64, queue_size=1)
		self.right_elbow_pronation = rospy.Publisher("/mannequin/right_elbow_pronation/command", Float64, queue_size=1)
		self.right_wrist_flexion = rospy.Publisher("/mannequin/right_wrist_flexion/command", Float64, queue_size=1)
		self.right_wrist_deviation = rospy.Publisher("/mannequin/right_wrist_deviation/command", Float64, queue_size=1)
		
		#legs
		#self.left_thigh = rospy.Publisher("/mannequin/left_thigh/command", Float64, queue_size=1)
		self.left_knee = rospy.Publisher("/mannequin/left_knee/command", Float64, queue_size=1)
		self.left_ankle = rospy.Publisher("/mannequin/left_ankle/command", Float64, queue_size=1)
		
		#self.right_thigh = rospy.Publisher("/mannequin/right_thigh/command", Float64, queue_size=1)
		self.right_knee = rospy.Publisher("/mannequin/right_knee/command", Float64, queue_size=1)
		self.right_ankle = rospy.Publisher("/mannequin/right_ankle/command", Float64, queue_size=1)
		
		#trunk parts
		#self.torso = rospy.Publisher("/mannequin/torso_lr/command", Float64, queue_size=1)
		self.head = rospy.Publisher("/mannequin/head_tilt/command", Float64, queue_size=1)
		
		self.init = Float64()
		
		
	def init_pose(self):
		# Return all joints to 0
		# legs
		"""self.left_thigh_pub_once(self.init)
		self.left_knee_pub_once(self.init)
		self.left_ankle_pub_once(self.init)
		
		self.right_thigh_pub_once(self.init)
		self.right_knee_pub_once(self.init)
		self.right_ankle_pub_once(self.init)
		
		# Midline
		self.torso_pub_once(self.init)
		self.head_pub_once(self.init)
		
		# arms
		self.left_shoulder_pub_once(self.init,self.init,self.init)
		self.left_elbow_pub_once(self.init,self.init)
		self.left_wrist_pub_once(self.init,self.init)
		
		self.right_shoulder_pub_once(self.init,self.init,self.init)
		self.right_elbow_pub_once(self.init,self.init)
		self.right_wrist_pub_once(self.init,self.init)"""
		rate = rospy.Rate(2)
		i = 0
		while not rospy.is_shutdown():
			self.left_thigh.publish(self.init)
			self.left_knee.publish(self.init)
			self.left_ankle.publish(self.init)
			
			self.right_thigh.publish(self.init)
			self.right_knee.publish(self.init)
			self.right_ankle.publish(self.init)
			
			self.torso.publish(self.init)
			self.head.publish(self.init)
			
			self.left_shoulder_roll.publish(self.init)
			self.left_shoulder_pitch.publish(self.init)
			self.left_shoulder_yaw.publish(self.init)
			self.left_elbow_flexion.publish(self.init)
			self.left_elbow_pronation.publish(self.init)
			self.left_wrist_flexion.publish(self.init)
			self.left_wrist_deviation.publish(self.init)
			
			self.right_shoulder_roll.publish(self.init)
			self.right_shoulder_pitch.publish(self.init)
			self.right_shoulder_yaw.publish(self.init)
			self.right_elbow_flexion.publish(self.init)
			self.right_elbow_pronation.publish(self.init)
			self.right_wrist_flexion.publish(self.init)
			self.right_wrist_deviation.publish(self.init)
			rate.sleep()
		
		
	#### Midline functions ####
	def torso_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.torso.publish(angle)
			if self.torso.get_num_connections() > 0:
				rospy.loginfo("Torso pos updated: p=%d",angle.data)
				break
	
	def head_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.head.publish(angle)
			if self.head.get_num_connections() > 0:
				rospy.loginfo("Head pos updated: p=%d",angle.data)
				break
		
		
	
	#### Left side functions ####
	# Left arm pub once functions
	def left_shoulder_pub_once(self,roll,pitch,yaw):
		while not rospy.is_shutdown():
			self.left_shoulder_roll.publish(roll)
			if self.left_shoulder_roll.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.left_shoulder_pitch.publish(pitch)
			if self.left_shoulder_pitch.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.left_shoulder_yaw.publish(yaw)
			if self.left_shoulder_yaw.get_num_connections() > 0:
				rospy.loginfo("Left shoulder pos updated: r=%d  p=%d  y:%d",roll.data,pitch.data,yaw.data)
				break
				
	
		
	def left_elbow_pub_once(self, flexion, pronation):
		while not rospy.is_shutdown():
			self.left_elbow_pronation.publish(pronation)
			if self.left_elbow_pronation.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.left_elbow_flexion.publish(flexion)
			if self.left_elbow_flexion.get_num_connections() > 0:
				rospy.loginfo("Left elbow pos updated: flex=%d  pro=%d",flexion.data,pronation.data)
				break
				
	
		
	def left_wrist_pub_once(self, flexion, deviation):
		while not rospy.is_shutdown():
			self.left_wrist_deviation.publish(deviation)
			if self.left_wrist_deviation.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.left_wrist_flexion.publish(flexion)
			if self.left_wrist_flexion.get_num_connections() > 0:
				rospy.loginfo("Left wrist pos updated: flex=%d  dev=%d",flexion.data,deviation.data)
				break
				
	
		
	def left_thigh_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.left_thigh.publish(angle)
			if self.left_thigh.get_num_connections() > 0:
				rospy.loginfo("Left thigh pos updated: p=%d",angle.data)
				break
	
		
	def left_knee_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.left_knee.publish(angle)
			if self.left_knee.get_num_connections() > 0:
				rospy.loginfo("Left knee pos updated: p=%d",angle.data)
				break
	
		
	def left_ankle_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.left_ankle.publish(angle)
			if self.left_ankle.get_num_connections() > 0:
				rospy.loginfo("Left ankle pos updated: p=%d",angle.data)
				break
	
		
		
	#### Right side functions ####
    # Right arm pub once functions
	def right_shoulder_pub_once(self,roll,pitch):
		while not rospy.is_shutdown():
			self.right_shoulder_roll.publish(roll)
			if self.right_shoulder_roll.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.right_shoulder_pitch.publish(pitch)
			if self.right_shoulder_pitch.get_num_connections() > 0:
				break
		"""while not rospy.is_shutdown():
			self.right_shoulder_yaw.publish(yaw)
			if self.right_shoulder_yaw.get_num_connections() > 0:
				rospy.loginfo("Right shoulder pos updated: r=%d  p=%d  y:%d",roll.data,pitch.data,yaw.data)
				break"""
				
	
		
	def right_elbow_pub_once(self, flexion, pronation):
		while not rospy.is_shutdown():
			self.right_elbow_pronation.publish(pronation)
			if self.right_elbow_pronation.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.right_elbow_flexion.publish(flexion)
			if self.right_elbow_flexion.get_num_connections() > 0:
				rospy.loginfo("Right elbow pos updated: flex=%d  pro=%d",flexion.data,pronation.data)
				break
				
	
		
	def right_wrist_pub_once(self, flexion, deviation):
		while not rospy.is_shutdown():
			self.right_wrist_deviation.publish(deviation)
			if self.right_wrist_deviation.get_num_connections() > 0:
				break
		while not rospy.is_shutdown():
			self.right_wrist_flexion.publish(flexion)
			if self.right_wrist_flexion.get_num_connections() > 0:
				rospy.loginfo("Right wrist pos updated: flex=%d  dev=%d",flexion.data,deviation.data)
				break
				
	
		
	def right_thigh_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.right_thigh.publish(angle)
			if self.right_thigh.get_num_connections() > 0:
				rospy.loginfo("Right thigh pos updated: p=%d",angle.data)
				break
	
		
	def right_knee_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.right_knee.publish(angle)
			if self.right_knee.get_num_connections() > 0:
				rospy.loginfo("Right knee pos updated: p=%d",angle.data)
				break
	
		
	def right_ankle_pub_once(self,angle):
		while not rospy.is_shutdown():
			self.right_ankle.publish(angle)
			if self.right_ankle.get_num_connections() > 0:
				rospy.loginfo("Right ankle pos updated: p=%d",angle.data)
				break
	
		
		
if __name__ == "__main__":
	rospy.init_node("move_mannequin", anonymous=True)
	mannequin = MoveMannequin()
	mannequin.init_pose()
	
		
		
		
		
		
		
		
