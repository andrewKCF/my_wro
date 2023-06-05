#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Duration
from std_msgs.msg import Bool

flag_enable=False

def publish_velocity(distance, max_velocity, max_acceleration, time_interval):
    #pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #rospy.init_node('trajectory_publisher', anonymous=True)
    #rate = rospy.Rate(100)  # 發佈頻率 (10 Hz)

    acceleration_time = max_velocity / max_acceleration
    deceleration_time = max_velocity / max_acceleration
    constant_velocity_time = (distance - max_velocity * acceleration_time - max_velocity * deceleration_time) / max_velocity
    total_time = acceleration_time + constant_velocity_time + deceleration_time

    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(total_time)

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
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

def CB_trj_enable(data):
    if data.data=="enable":
        flag_enable=1
    if data.data=="disable":
        flag_enable=0  

if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_publisher', anonymous=True)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(100)  # 發佈頻率 (10 Hz)  
        #rospy.Subscriber("trj_enable", String, CB_trj_enable,queue_size=1,buff_size=65536)
        distance = 3.0  # 距離 (米)
        max_velocity = 0.5  # 最大速度 (米/秒)
        max_acceleration = 0.2  # 最大加速度 (米/秒^2)
        time_interval = 0.001  # 發佈頻率 (秒)
        while not rospy.is_shutdown():
            #rospy.loginfo('/trj_enable:%s',flag_enable)
            try:
                flag_enable = rospy.wait_for_message('/trj_enable',  Bool,timeout=0.1).data
            except:
                pass
            rospy.loginfo('/trj_enable:%s',flag_enable)
            if flag_enable:
                rospy.loginfo("Publishing trajectory...")
                publish_velocity(distance, max_velocity, max_acceleration, time_interval)
                rospy.loginfo("Trajectory published.")
                flag_enable=False
    except rospy.ROSInterruptException:
        pass
