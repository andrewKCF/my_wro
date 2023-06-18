#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from datetime import datetime
from my_wro.srv import *
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Bool # setpoint state
from std_msgs.msg import String # setpoint state
from std_msgs.msg import Int16
setMotor=None
rospy.wait_for_service('titan/set_motor_speed')
setMotor=rospy.ServiceProxy('titan/set_motor_speed', MotorSpeed, persistent=True, headers=None)
control_effort=0
control_effort_pre=0
pub_state=None

def callback(data):
    global setMotor 
    global control_effort,control_effort_pre
    #if rospy.get_param('~motor_num')=='0' or rospy.get_param('~motor_num')=='1':
        #data.data=data.data
    #else:
    data.data=-data.data
    
    control_effort=control_effort_pre+data.data
    control_effort_pre=control_effort
    
    if control_effort <=-1:
      control_effort=-1
      '''
    if control_effort >=0:
      control_effort=0'''
    if control_effort >=1:
      control_effort=1    
    
    #setMotor(0,data.data) #motor_num  speed
    setMotor(int(rospy.get_param('~motor_num')),control_effort)
    rospy.loginfo("motor num:"+str(rospy.get_param('~motor_num')))
    rospy.loginfo("abs control_effort:"+str(data.data))
    rospy.loginfo("rel control_effort:"+str(control_effort))
    #setMotor(0,-1) 

#pub_state = rospy.Publisher('/motor'+str(rospy.get_param('~motor_num'))+'/state', Float64, queue_size=1)
#pub_state = rospy.Publisher('/state', Float64, queue_size=1)
def convert_callback(data):
    global pub_state
    float_data = float(data.data)
    float_msg = Float64()
    float_msg.data =float_data
    rospy.loginfo("sate:"+str(float_msg.data))
    pub_state.publish(float_msg.data)

  

# Intializes everything

def start():
    global pub_state
    rospy.init_node('control_effort')
    str_sub_titan_rpm="/titan/motor"+str(rospy.get_param('~motor_num'))+"/rpm"
    str_remap="/motor"+str(rospy.get_param('~motor_num'))
    str_state=str_remap+"/state"
    str_setpoint=str_remap+"/setpoint"
    str_control_effort=str_remap+"/control_effort"
    str_pid_enable=str_remap+"/pid_enable"
    
    #rospy.Subscriber("control_effort", Float64, callback,queue_size=1,buff_size=65536)
    #rospy.Subscriber("/titan/motor0/rpm", Int16, convert_callback,queue_size=1,buff_size=65536)
    #pub_setpoint = rospy.Publisher('setpoint', Float64, queue_size=1)
    #pub_enable = rospy.Publisher('pid_enable', Bool, queue_size=1)
    rospy.Subscriber(str_control_effort, Float64, callback,queue_size=1,buff_size=65536)
    rospy.Subscriber(str_sub_titan_rpm, Int16, convert_callback,queue_size=1,buff_size=65536)
    pub_state = rospy.Publisher(str_state, Float64, queue_size=1)
    pub_setpoint = rospy.Publisher(str_setpoint, Float64, queue_size=1)
    pub_enable = rospy.Publisher(str_pid_enable, Bool, queue_size=1)
    pub_enable.publish(1)
    rate = rospy.Rate(100) # 10hz
    '''
    while not rospy.is_shutdown():
        pub_setpoint.publish(int(rospy.get_param('~set_point')))
        #pub_setpoint.publish(-40)
        #rospy.loginfo("setpoint:"+str(40))
        #pub_state.publish(70)
        rate.sleep()
    '''
    rospy.spin()

if __name__ == '__main__':
    print("control effort output")
    start()

   
'''
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("chatter", String, callback)
        talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
'''