#! /usr/bin/env python3

""" Simplified way to publish shoulder positions. Could be expanded to control multiple joints """

import rospy
import sys
from std_msgs.msg import Float64
from move_mannequin import MoveMannequin


if __name__ == "__main__":
	rospy.init_node("move_mannequin", anonymous=True)
	mannequin = MoveMannequin()
	roll = Float64()
	pitch = Float64()
	yaw = Float64()
	while not rospy.is_shutdown():
		print("~~~~~~~~~~~~~~~~~~~~~~~~~")
		cont = input("Update shoulder position? (y/n): ")
		if cont == "n": # written like this so any key but n will continue program
			break
		roll.data = float(input("Enter shoulder ROLL: "))
		pitch.data = float(input("Enter shoulder PITCH: "))
		#yaw.data = float(input("Enter shoulder YAW: "))
		mannequin.right_shoulder_pub_once(roll,pitch)
		rospy.sleep(0.5)
	
	
	

