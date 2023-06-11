#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Duration
from std_msgs.msg import Bool

flag_enable=False
flag_complete=True

def publish_velocity(distance, max_velocity, max_acceleration, time_interval):
    global flag_enable,flag_complete
    acceleration_time = max_velocity / max_acceleration
    deceleration_time = max_velocity / max_acceleration
    constant_velocity_time = (distance - max_velocity * acceleration_time - max_velocity * deceleration_time) / max_velocity
    total_time = acceleration_time + constant_velocity_time + deceleration_time

    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(total_time)

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        flag_complete=False
        if flag_enable==False:
            flag_complete=True
            rospy.loginfo("exit trajectory")
            return
        elapsed_time = rospy.Time.now() - start_time

        if elapsed_time <= rospy.Duration(acceleration_time):
            # 加速段
            current_velocity = max_acceleration * elapsed_time.to_sec()
        elif elapsed_time <= rospy.Duration(acceleration_time + constant_velocity_time):
            # 等速段
            current_velocity = max_velocity
        else:
            # 減速段
            deceleration_elapsed_time = elapsed_time - rospy.Duration(acceleration_time + constant_velocity_time)
            current_velocity = max_velocity - max_acceleration * deceleration_elapsed_time.to_sec()

        # 發佈速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = current_velocity
        pub.publish(cmd_vel)

        rate.sleep()

    # 停止機器人
    cmd_vel = Twist()
    pub.publish(cmd_vel)
    flag_complete=True

def CB_trj_enable(data):
    global flag_enable,flag_complete
    if data.data==1:
        flag_enable=1
        rospy.loginfo('/trj_enable:%s',flag_enable)
        distance = 10.0  # 距離 (米)
        max_velocity = 0.5  # 最大速度 (米/秒)
        max_acceleration = 0.2  # 最大加速度 (米/秒^2)
        time_interval = 0.001  # 發佈頻率 (秒) 
        if flag_complete:
            publish_velocity(distance, max_velocity, max_acceleration, time_interval)
    if data.data==0:
        flag_enable=0
        rospy.loginfo('/trj_enable:%s',flag_enable)

if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_publisher', anonymous=True)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(100)  # 發佈頻率 (10 Hz)  
        rospy.Subscriber("trj_enable", Bool, CB_trj_enable,queue_size=1,buff_size=65536)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
