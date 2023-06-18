#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

enable = False

def enable_callback(data):
    global enable
    enable = data.data
    print("enter callback")

def generate_cmd_vel():
    global enable
    rate = rospy.Rate(100)  # 指定計時器頻率 (100Hz)

    while not rospy.is_shutdown():
        if enable:
            # 設定加速度和目標速度
            acceleration = 0.1  # 加速度 (m/s^2)
            target_speed = 1.0  # 目標速度 (m/s)

            # 計算加速度階段的時間和距離
            acceleration_time = target_speed / acceleration
            acceleration_distance = 0.5 * acceleration * acceleration_time**2

            # 計算維持目標速度的時間
            constant_speed_time = 2.0 * acceleration_distance / target_speed

            # 計算總時間
            total_time = 2.0 * acceleration_time + constant_speed_time

            start_time = rospy.get_time()

            while not rospy.is_shutdown():
                current_time = rospy.get_time()
                elapsed_time = current_time - start_time

                # 根據時間計算線速度和角速度
                if elapsed_time <= acceleration_time:
                    # 加速階段
                    linear_velocity = acceleration * elapsed_time
                    angular_velocity = 0.0
                elif elapsed_time <= (total_time - acceleration_time):
                    # 維持目標速度階段
                    linear_velocity = target_speed
                    angular_velocity = 0.0
                else:
                    # 減速階段
                    remaining_time = total_time - elapsed_time
                    linear_velocity = acceleration * remaining_time
                    angular_velocity = 0.0

                # 建立並發佈cmd_vel指令
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_velocity
                cmd_vel.angular.z = angular_velocity
                print("vel")
                pub.publish(cmd_vel)

                # 檢查是否達到總時間，若是則停止
                if elapsed_time >= total_time:
                    break

                rate.sleep()
        else:
            # 若enable為False，以100Hz發佈零速度的/cmd_vel指令
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            print("vel=0")
            pub.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('trapezoidal_acceleration')

    # 訂閱enable主題，指定回呼函式為enable_callback
    rospy.Subscriber('/enable', Bool, enable_callback)

    # 發佈cmd_vel主題
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    generate_cmd_vel()
