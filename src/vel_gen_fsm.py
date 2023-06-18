#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

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

    def execute(self, userdata):
        # 在这里编写进入 TrapVelocity 状态时的逻辑
        rospy.loginfo("Entering TrapVelocity state")
        rospy.loginfo(self.enable_data)
        rospy.sleep(2)  # 假设需要停留2秒
        if self.enable_data:
            rospy.loginfo("stay state")
            return 'success'
        else:
            rospy.loginfo("Exiting TrapVelocity state")
            return 'zero_vel'
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
        rospy.sleep(2)  # 假设需要停留2秒
        if self.enable_data:
            rospy.loginfo("Exiting ZeroVelocity state")
            return 'trap_vel'
        else:
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
