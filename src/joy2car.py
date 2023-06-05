#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from datetime import datetime
from my_wro.srv import *
setMotor=None
rospy.wait_for_service('titan/set_motor_speed')
setMotor=rospy.ServiceProxy('titan/set_motor_speed', MotorSpeed, persistent=True, headers=None)

def callback(data):
        global setMotor 
        print('stick left LR 0:%s'%data.axes[0])
        print('stick left UD 1:%s'%data.axes[1])
        print('stick right LR 2:%s'%data.axes[2])                
        print('stick right UD 3:%s'%data.axes[3])                
        print('R2:%s'%data.axes[4])
        print('L2:%s'%data.axes[5])
        print('cross key LR:{:.50f}'.format(data.axes[6]))
        print('cross key UD:%s'%data.axes[7])
        

        print('A:%s'%data.buttons[0])
        print('B:%s'%data.buttons[1])
        #print('2:%s'%data.buttons[2])
        print('X:%s'%data.buttons[3])
        print('Y:%s'%data.buttons[4])
        #print('5:%s'%data.buttons[5])
        print('L1:%s'%data.buttons[6])
        print('R1:%s'%data.buttons[7])
        print('L2:%s'%data.buttons[8])
        print('R2:%s'%data.buttons[9])
        print('SELECT:%s'%data.buttons[10])
        print('START:%s'%data.buttons[11])
        #print('12:%s'%data.buttons[12])
        print('stick left bu:%s'%data.buttons[13])
        print('stick right bu:%s'%data.buttons[14])
        now=datetime.now()
        current_time=now.strftime("%H:%M:%S")
        print("current time:",current_time)
        if (data.axes[7]>0):
                print("forward")
                setMotor(0,1) #motor_num  speed
                setMotor(1,1) #motor_num  speed
                setMotor(2,-1) #motor_num  speed
                setMotor(3,-1) #motor_num  speed
                return
        elif(data.axes[7]<0):
                print("backward")
                setMotor(0,-1) #motor_num  speed
                setMotor(1,-1) #motor_num  speed
                setMotor(2,1) #motor_num  speed
                setMotor(3,1) #motor_num  speed
                return
                

        if (data.axes[6]>0):
                print("left")
                setMotor(0,-0.4) #motor_num  speed
                setMotor(1,-0.4) #motor_num  speed
                setMotor(2,-0.4) #motor_num  speed
                setMotor(3,-0.4) #motor_num  speed
                return
        elif(data.axes[6]<0):
                print("right")
                setMotor(0,0.4) #motor_num  speed
                setMotor(1,0.4) #motor_num  speed
                setMotor(2,0.4) #motor_num  speed
                setMotor(3,0.4) #motor_num  speed
                return
        
        if (data.buttons[3]==1): # x
                setMotor(0,0.5) #motor_num  speed
                setMotor(1,-0.5) #motor_num  speed
                setMotor(2,0.5) #motor_num  speed
                setMotor(3,-0.5) #motor_num  speed 
                return
                
        if (data.buttons[1]==1): # B
                setMotor(0,-0.5) #motor_num  speed
                setMotor(1,0.5) #motor_num  speed
                setMotor(2,-0.5) #motor_num  speed
                setMotor(3,0.5) #motor_num  speed 
                return

        setMotor(0,0) #motor_num  speed
        setMotor(1,0) #motor_num  speed
        setMotor(2,0) #motor_num  speed
        setMotor(3,0) #motor_num  speed 
        return
                        
'''
        else:
            print("STOP")
            ##setMotor(0,0) #motor_num  speed
            ##setMotor(1,0) #motor_num  speed
            ##setMotor(2,0) #motor_num  speed
            #3setMotor(3,0) #motor_num  speed  
'''            


def start():
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback,queue_size=1,buff_size=65536)
	# starts the node
	rospy.init_node('joy2car')
	rospy.spin()

if __name__ == '__main__':
   print("joy2car START")
   start()