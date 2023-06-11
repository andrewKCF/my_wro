#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from sensor_msgs.msg import Joy

# 定義狀態A
class StateA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['button_pressed'])
        self.button_pressed = False

    def execute(self, userdata):
        rospy.loginfo("state Manul")
        rospy.loginfo("wait pressed L1...")
        while not self.button_pressed:
        #while not self.button_pressed:
            #rospy.sleep(0.1)  # 延遲0.1秒，等待按鈕被按下
            rospy.loginfo(self.button_pressed)
        #self.button_pressed = False  # 重置按鈕狀態
        return 'button_pressed'

# 定義狀態B
class StateB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['button_released'])
        self.button_released = False

    def execute(self, userdata):
        rospy.loginfo("state Auto")
        rospy.loginfo("waite release...")
        while not self.button_released:
            rospy.sleep(0.1)  # 延遲0.1秒，等待按鈕被放開
        self.button_released = False  # 重置按鈕狀態
        return 'button_released'

state_a = StateA()
state_b = StateB()
button_pressed=False

def joy_callback(data):
    global button_pressed
    # 在這裡獲取按鈕狀態並更新狀態機的用戶數據
    if data.buttons[6] == 1: #L1
        button_pressed=not(button_pressed)
        state_a.button_pressed = button_pressed
        #rospy.loginfo(state_a.button_pressed)
    '''
    elif data.buttons[6] == 0:
        state_b.button_released = True'''

def main():
    rospy.init_node('joy_state_machine')

    # 創建有限狀態機
    sm = smach.StateMachine(outcomes=['success'])

    with sm:
        # 添加狀態A
        smach.StateMachine.add('STATE_A', state_a,
                               transitions={'button_pressed': 'STATE_B'})

        # 添加狀態B
        smach.StateMachine.add('STATE_B', state_b,
                               transitions={'button_released': 'STATE_A'})

    # 創建ROS Joy的訂閱者
    rospy.Subscriber("joy", Joy, joy_callback)

    # 創建SMACH介面
    sis = smach_ros.IntrospectionServer('joy_state_machine', sm, '/SM_ROOT')
    sis.start()

    # 開始執行有限狀態機
    outcome = sm.execute()

    # 停止SMACH介面
    sis.stop()

if __name__ == '__main__':
    main()
