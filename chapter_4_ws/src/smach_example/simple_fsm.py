#!/usr/bin/env python
import rospy
from smach import State,StateMachine
from time import sleep
import smach_ros

class A(State):
  def __init__(self):
    State.__init__(self, outcomes=['1','0'], input_keys=['input'], output_keys=[''])
  
  def execute(self, userdata):
    sleep(1)
    if userdata.input == 1:
      return '1'
    else:
      return '0'

class B(State):
  def __init__(self):
    State.__init__(self, outcomes=['1','0'], input_keys=['input'], output_keys=[''])
  def execute(self, userdata):
    sleep(1)
    if userdata.input == 1:
      return '1'
    else:
      return '0'

class C(State):
  def __init__(self):
    State.__init__(self, outcomes=['1','0'], input_keys=['input'], output_keys=[''])
  def execute(self, userdata):
    sleep(1)
    if userdata.input == 1:
      return '1'
    else:
      return '0'

class D(State):
  def __init__(self):
    State.__init__(self, outcomes=['1','0'], input_keys=['input'], output_keys=[''])
  def execute(self, userdata):
    sleep(1)
    if userdata.input == 1:
      return '1'
    else:
      return '0'

if __name__ == '__main__':

  rospy.init_node('test_fsm', anonymous=True)
  sm = StateMachine(outcomes=['success'])
  sm.userdata.sm_input = 1
  with sm:
    StateMachine.add('A', A(), transitions={'1':'B','0':'D'}, remapping={'input':'sm_input','output':'input'})
    StateMachine.add('B', B(), transitions={'1':'C','0':'A'}, remapping={'input':'sm_input','output':'input'})
    StateMachine.add('C', C(), transitions={'1':'D','0':'B'}, remapping={'input':'sm_input','output':'input'})
    StateMachine.add('D', D(), transitions={'1':'A','0':'C'}, remapping={'input':'sm_input','output':'input'})
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  sm.execute()
  rospy.spin()
  sis.stop()
