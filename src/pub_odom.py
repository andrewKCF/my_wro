#!/usr/bin/env python3
# coding: utf-8
import math
import rospy
import tf
from nav_msgs.msg import Odometry
import os
#from std_msgs.msg import Int64
from std_msgs.msg import Int32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
#from std_srvs.srv import Empty
#from my_wro.srv import Empty
#rospy.wait_for_service('/titan/reset_encoder')
#reset_encoder_service = rospy.ServiceProxy('/titan/reset_encoder', Empty)
#reset_encoder_service()
os.system("rosservice call /titan/reset_encoder")

#Parameters
R = 0.05 #unit:meter wheelradius
TPR = 1464
#TPR = 1440
#thetaPtick=0.004289 #(2*math.pi)/TPR
thetaPtick=(2*math.pi)/TPR
distancePerTick=0.000218881455 #unit:meter
L1=0.384/2 #unit:meter 
L2=0.192/2 #unit:meter 

LF_ticks = 0
RF_ticks = 0
LR_ticks = 0
RR_ticks = 0

last_LF_ticks = 0
last_RF_ticks = 0
last_LR_ticks = 0
last_RR_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

def LFCB(msg):
    global LF_ticks 
    LF_ticks = msg.data
    #rospy.loginfo("LFCB")

def RFCB(msg):
    global RF_ticks 
    RF_ticks = -msg.data
    #rospy.loginfo("RFCB")

def LRCB(msg):
    global LR_ticks 
    LR_ticks =msg.data
    #rospy.loginfo("LRCB")


def RRCB(msg):
    global RR_ticks 
    RR_ticks = -msg.data
    #rospy.loginfo("RRCB")

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

LF_ticks_sub =rospy.Subscriber("/titan/encoder1/count", Int32, LFCB,queue_size=1,buff_size=65536)
RF_ticks_sub =rospy.Subscriber("/titan/encoder3/count", Int32, RFCB,queue_size=1,buff_size=65536)
LR_ticks_sub =rospy.Subscriber("/titan/encoder0/count", Int32, LRCB,queue_size=1,buff_size=65536)
RR_ticks_sub =rospy.Subscriber("/titan/encoder2/count", Int32, RRCB,queue_size=1,buff_size=65536)

odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)

coor_name_fix="ins_frame"
coor_name_dynamic="mobile_frame"

x_LF=0
x_RF=0
x_LR=0
x_RR=0
Xb=0
Yb=0
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    delta_LF = (LF_ticks - last_LF_ticks)*thetaPtick # rad
    delta_RF = (RF_ticks - last_RF_ticks)*thetaPtick
    delta_LR = (LR_ticks - last_LR_ticks)*thetaPtick
    delta_RR = (RR_ticks - last_RR_ticks)*thetaPtick

    dt = (current_time - last_time).to_sec()
    w1=delta_LF/dt
    w2=delta_RF/dt
    w3=delta_LR/dt
    w4=delta_RR/dt

    v_LF=R*w1
    x_LF=x_LF+v_LF*dt

    v_RF=R*w2
    x_RF=x_RF+v_RF*dt

    v_LR=R*w3
    x_LR=x_LR+v_LR*dt

    v_RR=R*w4
    x_RR=x_RR+v_RR*dt

    Xb=(x_LF + x_RF + x_LR + x_RR)/4
    Yb=(-x_LF + x_RF + x_LR - x_RR)/4
    
    vxb=R*(w1 + w2 + w3 + w4)/4
    vyb=R*(-w1 + w2 + w3 - w4)/4
    #vyb=R*(w1 - w2 - w3 + w4)/4    
    vthb=R*(-w1 + w2 - w3 + w4)/(4*(L1 + L2))
    vth=vthb
 
    th+=vth*dt
    th=th%(2*math.pi)
    th_degree=th*180/math.pi
    vx = math.cos(th)*vxb - math.sin(th)*vyb
    vy = math.sin(th)*vxb + math.cos(th)*vyb
    x+=vx*dt
    y+=vy*dt    
    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    print("----------------------------")
    #print("thetaPtick:%s"%thetaPtick)
    #print("TPR :%s"%TPR )
    #print("delta_LF :%s"%delta_LF)
    #print("w1 :%s"%w1)
    #print("w2 :%s"%w2)
    #print("w3 :%s"%w3)
    #print("w4 :%s"%w4)
    print("th :%s"% th_degree)
    print("vxb :%s"%vxb)
    print("vyb :%s"%vyb)
    print("thetaPtick :%s"%thetaPtick)
    '''
    print("vthb :%s"%vthb)
    print("rho :%s"%rho)
    print("vx :%s"%vx)
    print("vy :%s"%vy)
    print("x :%s"%x)
    print("y :%s"%y)
    '''


    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       coor_name_dynamic,
       coor_name_fix
    )

    odom_broadcaster.sendTransform(
       (1.2, 0, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "fix_sign",
       coor_name_fix
    )

    odom_broadcaster.sendTransform(
       (R*thetaPtick*RF_ticks, 0, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "RF_tick",
       coor_name_fix
    )

    odom_broadcaster.sendTransform(
       (R*thetaPtick*LF_ticks, 0, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "LF_tick",
       coor_name_fix
    )

    odom_broadcaster.sendTransform(
       (R*thetaPtick*LR_ticks, 0, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "LR_tick",
       coor_name_fix
    )

    odom_broadcaster.sendTransform(
       (R*thetaPtick*RR_ticks, 0, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "RR_tick",
       coor_name_fix
    )


    odom_broadcaster.sendTransform(
       (Xb, Yb, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "tick_frame",
       coor_name_fix
    )

	
    odom_broadcaster.sendTransform(
       (x_RF, 0, 0.),
       tf.transformations.quaternion_from_euler(0, 0, 0),
       current_time,
       "RF_V",
       coor_name_fix
    )	

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = coor_name_fix
    odom.child_frame_id = coor_name_dynamic
    '''
    odom.child_frame_id = coor_name_dynamic
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)
    '''
    odom.pose.pose.position.x = x  # X軸方向的位移
    odom.pose.pose.position.y = y  # Y軸方向的位移
    odom.pose.pose.position.z = 0.0  # Z軸方向的位移
    
    # 計算四元數表示的姿態
    #roll = 0.0  # 繞X軸的尤拉角
    #pitch = 0.0  # 繞Y軸的尤拉角
    #yaw = math.pi / 4  # 繞Z軸的尤拉角
    #quaternion = quaternion_from_euler(roll, pitch, yaw)
    odom.pose.pose.orientation = Quaternion(*odom_quat)

    # 設定twist訊息的內容
    odom.twist.twist.linear.x = vxb  # X軸方向的線速度
    odom.twist.twist.linear.y = vyb  # Y軸方向的線速度
    odom.twist.twist.linear.z = 0.0  # Z軸方向的線速度
    odom.twist.twist.angular.x = 0.0  # X軸方向的角速度
    odom.twist.twist.angular.y = 0.0  # Y軸方向的角速度
    odom.twist.twist.angular.z = vth  # Z軸方向的角速度    
    odom_pub.publish(odom)

    #update 
    last_LF_ticks = LF_ticks
    last_RF_ticks = RF_ticks
    last_LR_ticks = LR_ticks
    last_RR_ticks = RR_ticks
    last_time = current_time
    r.sleep()