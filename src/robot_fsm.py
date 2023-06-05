#!/usr/bin/env python3
import rospy
from smach import State,StateMachine
from time import sleep
import smach_ros

class Manual(State):
  def __init__(self):
    State.__init__(self, outcomes=['controller_on','manual','navigation'], input_keys=['input'], output_keys=[''])
  
  def execute(self, userdata):
    sleep(1)
    return 'controller_on'
    '''
    if userdata.input == 'controller_on':
      return 'controller_on'
    else:
      return 'manual'
    '''

class Controller_On(State):
  def __init__(self):
    State.__init__(self, outcomes=['controller_on','manual','navigation'], input_keys=['input'], output_keys=[''])
  def execute(self, userdata):
    sleep(1)
    return 'navigation'
    '''
    if userdata.input == 'manual':
      return 'manual'
    else:
      return 'controller_on'
    '''

class Navigation(State):
  def __init__(self):
    State.__init__(self, outcomes=['controller_on','manual','navigation'], input_keys=['input'], output_keys=[''])
  def execute(self, userdata):
    sleep(1)
    return 'manual'

 
if __name__ == '__main__':

  rospy.init_node('robot_fsm', anonymous=True)
  sm = StateMachine(outcomes=['controller_on','manual','navigation'])
  sm.userdata.sm_input = 'manual'
  with sm:
    StateMachine.add('MANUAL', Manual(), transitions={'controller_on':'CONTROLLER_ON','navigation':'NAVIGATION'}, remapping={'input':'sm_input','output':'input'})
    StateMachine.add('CONTROLLER_ON', Controller_On(), transitions={'navigation':'NAVIGATION','manual':'MANUAL'}, remapping={'input':'sm_input','output':'input'})
    StateMachine.add('NAVIGATION', Navigation(), transitions={'manual':'MANUAL'}, remapping={'input':'sm_input','output':'input'})
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  sm.execute()
  rospy.spin()
  sis.stop()