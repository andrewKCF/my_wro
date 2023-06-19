#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


def publish_velocity(distance, max_velocity, max_acceleration, time_interval):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)  # 發佈頻率 (10 Hz)  
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

# 定义 TrapVelocity 状态
class TrapVelocity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','zero_vel'])
        self.subscriber = rospy.Subscriber('trj_enable', Bool, self.callback)
        self.enable_data = None

    def callback(self, data):
        self.enable_data = data.data
        '''
        if data.data == False:
            # 收到 trj_enable=False 时，切换到 ZeroVelocity 状态
            self.subscriber.unregister()
            self.enable_data = False
            '''
    def publish_velocity(self,distance, max_velocity, max_acceleration, time_interval):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub_trj = rospy.Publisher('/trj_enable', Bool, queue_size=1)
        rate = rospy.Rate(100)  # 發佈頻率 (10 Hz)  
        acceleration_time = max_velocity / max_acceleration
        deceleration_time = max_velocity / max_acceleration
        constant_velocity_time = (distance - max_velocity * acceleration_time - max_velocity * deceleration_time) / max_velocity
        total_time = acceleration_time + constant_velocity_time + deceleration_time

        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(total_time)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time :
            if self.enable_data==False:
                break
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
        pub_trj.publish(0)

    def execute(self, userdata):
        # 在这里编写进入 TrapVelocity 状态时的逻辑
        rospy.loginfo("Entering TrapVelocity state")
        #rospy.loginfo(self.enable_data)
        #rospy.sleep(2)  # 假设需要停留2秒

        #if self.enable_data:
        distance = 10.0  # 距離 (米)
        max_velocity = 0.5  # 最大速度 (米/秒)
        max_acceleration = 0.2  # 最大加速度 (米/秒^2)
        time_interval = 0.001  # 發佈頻率 (秒) 
        #publish_velocity(distance, max_velocity, max_acceleration, time_interval)
        self.publish_velocity(distance, max_velocity, max_acceleration, time_interval)
        #return 'success'
        return 'zero_vel'
        #else:
            #rospy.loginfo("Exiting TrapVelocity state")
            #return 'zero_vel'
        '''
        # 执行所需的操作
        # ...

        rospy.sleep(5)  # 假设需要停留5秒

        # 完成状态，返回成功
        rospy.loginfo("Exiting TrapVelocity state")
        return 'success'
        '''

# 定义 ZeroVelocity 状态
class ZeroVelocity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','trap_vel'])
        self.subscriber = rospy.Subscriber('trj_enable', Bool, self.callback)
        self.enable_data = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(100)  # 發佈頻率 (10 Hz)
        self.cmd_vel = Twist()

    def callback(self, data):
        self.enable_data = data.data
        '''
        if data.data == True:
            # 收到 trj_enable=True 时，切换到 TrapVelocity 状态
            self.subscriber.unregister()
            #self.parent.set_initial_state(['TRAP_VELOCITY'])
            #self.enable_data = True
            '''  

    def execute(self, userdata):
        # 在这里编写进入 ZeroVelocity 状态时的逻辑
        rospy.loginfo("Entering ZeroVelocity state")
        rospy.loginfo(self.enable_data)
        #rospy.sleep(2)  # 假设需要停留2秒
        if self.enable_data:
            rospy.loginfo("Exiting ZeroVelocity state")
            return 'trap_vel'
        else:
            self.pub.publish(self.cmd_vel)
            self.rate.sleep()
            rospy.loginfo("stay state")
            return 'success'
        # 执行所需的操作
        # ...

        

        # 完成状态，返回成功
        '''
        rospy.loginfo("Exiting ZeroVelocity state")
        return 'success'
        '''

def main():
    rospy.init_node('smach_example')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['outcome'])

    # 添加 TrapVelocity 状态到状态机
    with sm:
        # 添加 ZeroVelocity 状态到状态机
        smach.StateMachine.add('ZERO_VELOCITY', ZeroVelocity(), transitions={'success':'ZERO_VELOCITY','trap_vel':'TRAP_VELOCITY'})    
        smach.StateMachine.add('TRAP_VELOCITY', TrapVelocity(), transitions={'success':'TRAP_VELOCITY','zero_vel':'ZERO_VELOCITY'})



    # 启动状态机
    outcome = sm.execute()

    # 打印最终状态
    rospy.loginfo("Final outcome: %s", outcome)

if __name__ == '__main__':
    main()
