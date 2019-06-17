#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import Int32, Bool
import actionlib
from battery_simulator.msg import battery_simAction, battery_simGoal, battery_simResult

def battery_state(charge_condition):
	goal = battery_simGoal()
	goal.charge_state = charge_condition
	client.send_goal(goal)

if __name__ == '__main__':
	rospy.init_node('BatterySimclient')
	client = actionlib.SimpleActionClient('battery_simulator', battery_simAction)
	client.wait_for_server()
	battery_state(int(sys.argv[1]))
	client.wait_for_result()
