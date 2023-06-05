#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Int16

pub_state = rospy.Publisher('state', Float64, queue_size=1)
def convert_callback(data):
    float_data = float(data.data)
    float_msg = Float64()
    float_msg.data =float_data
    rospy.loginfo("sate:"+str(float_msg.data))
    pub_state.publish(float_msg.data)
                  

# Intializes everything

def start():
    rospy.init_node('pub_state')
    rospy.Subscriber("/titan/motor0/rpm", Int16, convert_callback,queue_size=1,buff_size=65536)
    rospy.spin()

if __name__ == '__main__':
    print("publish state(rpm)")
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