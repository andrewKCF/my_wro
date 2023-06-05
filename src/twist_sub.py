#!/usr/bin/env python3
import rospy
import os
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(data):
   # data.linear.x
   # data.angular.z
   rospy.loginfo("Linear velocity in x direction: %f", data.linear.x)
   cmd_str="rosservice call /titan/set_motor_speed 0 " + str(data.linear.x)
   #os.system("rosservice call /titan/set_motor_speed 0 0.1")
   rospy.loginfo(cmd_str)
   os.system(cmd_str)
   pass
    
def listener():
   rospy.init_node('twist_subscriber')
   rospy.Subscriber("/cmd_vel", Twist, callback)
   rospy.spin()


if __name__ == '__main__':
    listener()
    
