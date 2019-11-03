#! /usr/bin/env python

import time
import rospy
from multiprocessing import Process
from std_msgs.msg import Int32, Bool
import actionlib
from battery_simulator.msg import battery_simAction, battery_simGoal, battery_simResult, battery_simFeedback

def batterySim():
  battery_level = 100
  result = battery_simResult()
  while not rospy.is_shutdown():
	if rospy.has_param("/MyRobot/BatteryStatus"):
		time.sleep(1)
		param = rospy.get_param("/MyRobot/BatteryStatus")
		if param == 1:
			if battery_level == 100:
				result.battery_status = "Full"
				server.set_succeeded(result)
				break
			else:								
				battery_level += 1
				rospy.loginfo("Charging...currently, %s", battery_level)
				time.sleep(4)	
		elif param == 0:
			battery_level -= 1
			rospy.logwarn("Discharging...currently, %s", battery_level)
			time.sleep(2)

def goalFun(goal):
  rate = rospy.Rate(2)
  process = Process(target = batterySim)
  process.start()
  time.sleep(1)
  if goal.charge_state == 0:
	rospy.set_param("/MyRobot/BatteryStatus",goal.charge_state)
  elif goal.charge_state == 1:
	rospy.set_param("/MyRobot/BatteryStatus",goal.charge_state)

if __name__ == '__main__':	
	rospy.init_node('BatterySimServer')
	server = actionlib.SimpleActionServer('battery_simulator', battery_simAction, goalFun, False)
	server.start()	
	rospy.spin()

