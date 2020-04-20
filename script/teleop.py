#!/usr/bin/env/ python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import math


class Teleop():

	def __init__(self):
		rospy.init_node('teleop')
		sub_joy = rospy.Subscriber('/joy', Joy, joyCallback)
		sub_twist = rospy.Subscriber('/twist_cmd', TwistStamped, twistCallback)
		pub_twist = rospy.Publisher('/carla/ego_vehicle/twist_cmd', Twist, queue_size=10)

def main():
	teleop = Teleop()

if __name__=="__main__":
	main()