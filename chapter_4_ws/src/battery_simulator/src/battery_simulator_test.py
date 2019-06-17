#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32, Bool

rospy.init_node('BatteryController_node')
pub = rospy.Publisher('BatteryStatus', Int32, queue_size=10)
rate = rospy.Rate(2)
battery_level = 100

while not rospy.is_shutdown():
	if rospy.has_param("/MyRobot/BatteryStatus"):
		param = rospy.get_param("/MyRobot/BatteryStatus")
		pub.publish(battery_level)
		if param == 1:
			print "Charging..."
			battery_level += 1
			time.sleep(4)
		elif param == 0:
			print "Discharging..."			
			battery_level -= 1
			time.sleep(2)
		rate.sleep()

