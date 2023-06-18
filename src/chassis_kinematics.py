#!/usr/bin/env python3
# coding: utf-8
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
rad2rpm=9.54929658551

def cmd_vel_callback(data):
    # 获取线速度和角速度
    vx = data.linear.x
    vy = data.linear.y
    wb = data.angular.z
    w1=-((vx-vy-(Lx+Ly)*wb)/R)*rad2rpm
    w2=-((vx+vy+(Lx+Ly)*wb)/R)*rad2rpm
    w3=((vx+vy-(Lx+Ly)*wb)/R)*rad2rpm
    w4=((vx-vy+(Lx+Ly)*wb)/R)*rad2rpm
    pub_setpoint_w1.publish(w1)
    pub_setpoint_w2.publish(w2)
    pub_setpoint_w3.publish(w3)
    pub_setpoint_w4.publish(w4)
    rospy.loginfo('w1:%s rpm',w1)
    rospy.loginfo('w2:%s rpm',w2)
    rospy.loginfo('w3:%s rpm',w3)
    rospy.loginfo('w4:%s rpm',w4)
    
    # 打印接收到的线速度和角速度
    #print("Linear velocity: {}".format(linear_vel))
    #print("Angular velocity: {}".format(angular_vel))
    
def subscriber_node():
    #rospy.loginfo('kinematics start')
    rospy.init_node('kinematics_node', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback,queue_size=1,buff_size=65536)
    rospy.spin()

if __name__ == '__main__':
    #Parameters
    R = 0.05 #unit:meter wheelradius
    Lx=0.384/2 #unit:meter 
    Ly=0.192/2 #unit:meter 
    rospy.loginfo('kinematics start')
    pub_setpoint_w1 = rospy.Publisher('/motor0/setpoint', Float64, queue_size=1)
    pub_setpoint_w2 = rospy.Publisher('/motor1/setpoint', Float64, queue_size=1)
    pub_setpoint_w3 = rospy.Publisher('/motor2/setpoint', Float64, queue_size=1)
    pub_setpoint_w4 = rospy.Publisher('/motor3/setpoint', Float64, queue_size=1)
    subscriber_node()